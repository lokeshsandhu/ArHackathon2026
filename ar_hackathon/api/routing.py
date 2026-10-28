"""Amazon Robotics Hackathon 2026 routing entry point.

Team name: Robot Warriors
"""

import heapq
import math
from collections import defaultdict
from functools import lru_cache
from itertools import permutations
from typing import Optional

from ar_hackathon.models.graph_state import GraphState

_graph_key = None
_adjacency = {}
_distances = {}
_previous_node = {}
_last_time = -1
_last_polled_unit = -1
_plan_time = -1
_plan_moves = {}
_NO_PATH = object()


def _travel_time(weight):
    return max(1, math.ceil(weight))


def _prepare_graph(state):
    global _graph_key, _adjacency, _distances, _previous_node, _last_time
    global _plan_time, _plan_moves
    key = (tuple((node.id, node.node_type, node.capacity) for node in state.nodes),
           tuple((e.from_node, e.to_node, e.weight, e.capacity,
                  e.bidirectional) for e in state.edges))
    if state.current_time_step < _last_time or key != _graph_key:
        _previous_node = {}
        _plan_time, _plan_moves = -1, {}
    _last_time = state.current_time_step
    if key == _graph_key:
        return _adjacency, _distances

    adjacency = {node.id: [] for node in state.nodes}
    for node in state.nodes:
        seen = set()
        for neighbor in state.neighbors(node.id):
            if neighbor in seen:
                continue
            seen.add(neighbor)
            # The engine uses the first matching edge when duplicates exist.
            edge = state.get_edge(node.id, neighbor)
            adjacency[node.id].append((neighbor, _travel_time(edge.weight), edge))

    distances = {}
    for source in adjacency:
        best = {source: 0}
        queue = [(0, source)]
        while queue:
            cost, node = heapq.heappop(queue)
            if cost != best[node]:
                continue
            for neighbor, edge_cost, _ in adjacency[node]:
                proposed = cost + edge_cost
                if proposed < best.get(neighbor, math.inf):
                    best[neighbor] = proposed
                    heapq.heappush(queue, (proposed, neighbor))
        distances[source] = best

    _graph_key, _adjacency, _distances = key, adjacency, distances
    _previous_node = {}
    return adjacency, distances


def _distance(distances, source, destination):
    return distances.get(source, {}).get(destination, math.inf)


def _position_and_delay(unit):
    if unit.in_transit:
        return unit.transit_destination, _travel_time(unit.transit_remaining_time)
    return unit.current_node, 0


def _delivery_target(state, unit, pod_by_id, distances, position, delay):
    """Choose the first dock in the highest-scoring short delivery order."""
    carried = [pod_by_id[pod_id] for pod_id in unit.carrying
               if pod_id in pod_by_id]
    stations = sorted({pod.destination_station for pod in carried})
    reachable = [station for station in stations
                 if math.isfinite(_distance(distances, position, station))]
    if not reachable:
        return None
    if len(reachable) > 4:
        return min(reachable,
                   key=lambda station: (_distance(distances, position, station), station))

    best = None
    for order in permutations(reachable):
        node, elapsed, gain = position, delay, 0.0
        for station in order:
            travel = _distance(distances, node, station)
            if not math.isfinite(travel):
                break
            elapsed += travel
            delivery_time = state.current_time_step + elapsed - 1
            gain += sum(math.exp(-(delivery_time - pod.entry_time) / 50)
                        for pod in carried if pod.destination_station == station)
            elapsed += 1  # Conservative turnaround allowance in this estimate.
            node = station
        else:
            candidate = (-gain, _distance(distances, position, order[0]), order)
            if best is None or candidate < best:
                best = candidate
    return (best[2][0] if best is not None else
            min(reachable,
                key=lambda station: (_distance(distances, position, station), station)))


def _delivery_plan_score(state, pods, distances, start, delay):
    """Return the best static delivery score and its final ready position."""
    stations = sorted({pod.destination_station for pod in pods})
    if not stations or len(stations) > 4:
        return None
    best = None
    for order in permutations(stations):
        node, elapsed, gain = start, delay, 0.0
        for station in order:
            travel = _distance(distances, node, station)
            if not math.isfinite(travel):
                break
            elapsed += travel
            delivery_time = state.current_time_step + elapsed - 1
            gain += sum(math.exp(-(delivery_time - pod.entry_time) / 50)
                        for pod in pods if pod.destination_station == station)
            elapsed += 1
            node = station
        else:
            candidate = (gain, -elapsed, order, node, elapsed)
            if best is None or candidate[:2] > best[:2]:
                best = candidate
    return (best[0], best[3], best[4]) if best is not None else None


def _insert_pickups(state, targets, waiting_by_source, pod_by_id,
                    distances):
    """Use a loaded robot's spare slot when pickup beats delivering first."""
    eligible = [unit for unit in state.drive_units
                if unit.carrying and unit.has_capacity]
    if len(eligible) * len(waiting_by_source) > 32:
        return
    pairs = []
    for unit in eligible:
        carried = [pod_by_id[pod_id] for pod_id in unit.carrying
                   if pod_id in pod_by_id]
        position, delay = _position_and_delay(unit)
        direct = _delivery_plan_score(state, carried, distances,
                                      position, delay)
        if direct is None:
            continue
        for source, waiting in waiting_by_source.items():
            pickup = _distance(distances, position, source)
            if not math.isfinite(pickup):
                continue
            bundle = waiting[:unit.capacity - len(unit.carrying)]
            if len({pod.destination_station for pod in carried + bundle}) > 4:
                continue
            combined = _delivery_plan_score(state, carried + bundle,
                                            distances, source,
                                            delay + pickup + 1)
            return_trip = _distance(distances, direct[1], source)
            if combined is None or not math.isfinite(return_trip):
                continue
            later = _delivery_plan_score(state, bundle, distances, source,
                                         direct[2] + return_trip + 1)
            if later is None:
                continue
            advantage = combined[0] - direct[0] - later[0]
            if advantage > 0.01:
                pairs.append((-advantage, pickup, unit.id, source))
    taken_units, taken_sources = set(), set()
    for _, _, unit_id, source in sorted(pairs):
        if unit_id not in taken_units and source not in taken_sources:
            targets[unit_id] = source
            taken_units.add(unit_id)
            taken_sources.add(source)


def _assign_targets(state, distances, adjacency):
    """Return one useful destination node for each unit."""
    targets = {}
    pod_by_id = {pod.id: pod for pod in state.active_pods}
    empty_units = []
    for unit in state.drive_units:
        position, delay = _position_and_delay(unit)
        if unit.carrying:
            target = _delivery_target(state, unit, pod_by_id,
                                      distances, position, delay)
            if target is not None:
                targets[unit.id] = target
        elif unit.has_capacity:
            empty_units.append((unit, position, delay))

    waiting_by_source = defaultdict(list)
    for pod in state.active_pods:
        if pod.carried_by is None and pod.current_node is not None:
            waiting_by_source[pod.current_node].append(pod)
    if not waiting_by_source:
        return targets
    if not empty_units:
        _insert_pickups(state, targets, waiting_by_source,
                        pod_by_id, distances)
        return targets

    # Arrival automatically picks up several pods at a source. Limit duplicate trips.
    max_capacity = max(unit.capacity for unit, _, _ in empty_units)
    tasks = []
    for source, pods in waiting_by_source.items():
        pods.sort(key=lambda pod: (pod.entry_time, pod.id))
        for slot in range(math.ceil(len(pods) / max_capacity)):
            sample = pods[min(slot * max_capacity, len(pods) - 1)]
            delivery = _distance(distances, source, sample.destination_station)
            if math.isfinite(delivery):
                tasks.append((source, slot, sample, delivery))

    path_aware = (len(state.drive_units) > 1 and max_capacity > 1 and
                  len(tasks) * len(empty_units) <= 32)
    traffic_aware = (len(state.drive_units) > 1 and max_capacity == 1 and
                     any(edge.capacity is not None for edge in state.edges) and
                     len(tasks) * len(empty_units) <= 32)
    nodes = ({node.id: node for node in state.nodes}
             if path_aware or traffic_aware else None)
    pairs = []
    for unit, position, delay in empty_units:
        observed = (_observed_resources(state, unit.id)
                    if path_aware or traffic_aware else None)
        for source, slot, pod, delivery in tasks:
            pickup = _distance(distances, position, source)
            if math.isfinite(pickup):
                if not path_aware:
                    age = max(0, state.current_time_step - pod.entry_time)
                    cost = delay + pickup + delivery - min(10, age * 0.1)
                    if traffic_aware:
                        route = _time_path(source, position, delay, adjacency,
                                           distances, observed, nodes)
                        if route is not None:
                            arrival = route[-1][1][0] if route else delay
                            cost += 0.75 * max(0, arrival - delay - pickup)
                else:
                    first = slot * max_capacity
                    bundle = waiting_by_source[source][first:first + unit.capacity]
                    cost = _feasible_pair_cost(state, source, bundle, position,
                                               delay, adjacency, distances,
                                               nodes, observed)
                    if cost is None:
                        continue
                pairs.append((cost, pickup, unit.id, source, slot))
    pairs.sort()
    # Solve the small assignment problem globally. Greedy pairs can consume a
    # unit that is the only good choice for a different storage location.
    if len(tasks) <= 12 and len(empty_units) <= 6:
        task_ids = sorted((source, slot) for source, slot, _, _ in tasks)
        units = sorted(unit.id for unit, _, _ in empty_units)
        pair_costs = {(unit_id, (source, slot)): cost
                      for cost, _, unit_id, source, slot in pairs}
        unassigned = len(task_ids)

        @lru_cache(None)
        def solve(unit_index, used):
            if unit_index == len(units):
                return (0, 0.0, ())
            rest = solve(unit_index + 1, used)
            best = (rest[0], rest[1], (unassigned,) + rest[2])
            for task_index, task_id in enumerate(task_ids):
                if used & (1 << task_index):
                    continue
                cost = pair_costs.get((units[unit_index], task_id))
                if cost is None:
                    continue
                rest = solve(unit_index + 1, used | (1 << task_index))
                candidate = (rest[0] + 1, rest[1] + cost,
                             (task_index,) + rest[2])
                if (candidate[0] > best[0] or
                    (candidate[0] == best[0] and
                     (candidate[1], candidate[2]) < (best[1], best[2]))):
                    best = candidate
            return best

        for unit_id, task_index in zip(units, solve(0, 0)[2]):
            if task_index != unassigned:
                targets[unit_id] = task_ids[task_index][0]
    else:
        taken_units, taken_tasks = set(), set()
        for _, _, unit_id, source, slot in pairs:
            task = (source, slot)
            if unit_id not in taken_units and task not in taken_tasks:
                targets[unit_id] = source
                taken_units.add(unit_id)
                taken_tasks.add(task)
    unclaimed = {source: pods for source, pods in waiting_by_source.items()
                 if source not in targets.values()}
    if unclaimed:
        _insert_pickups(state, targets, unclaimed, pod_by_id, distances)
    return targets


def _legal_move(state, unit, neighbor, edge):
    if edge.capacity is not None and state.edge_occupancy(unit.current_node, neighbor) >= edge.capacity:
        return False
    node = state.get_node(neighbor)
    if node is not None and node.capacity is not None and state.node_occupancy(neighbor) >= node.capacity:
        return False
    return True


def _blocked_wait(state, origin, neighbor, edge):
    waits = []
    if edge.capacity is not None and state.edge_occupancy(origin, neighbor) >= edge.capacity:
        waits.extend(unit.transit_remaining_time for unit in state.drive_units
                     if unit.in_transit and edge.connects(unit.current_node, unit.transit_destination))
    node = state.get_node(neighbor)
    if node is not None and node.capacity is not None and state.node_occupancy(neighbor) >= node.capacity:
        waits.extend(unit.transit_remaining_time for unit in state.drive_units
                     if unit.in_transit and unit.transit_destination == neighbor)
        if any(not unit.in_transit and unit.current_node == neighbor for unit in state.drive_units):
            waits.append(2)
    return min(waits) if waits else 2


def _choose_move(unit, target, state, adjacency, distances):
    origin = unit.current_node
    if target is None or target == origin:
        # Release a limited-capacity station after delivery.
        node = state.get_node(origin)
        if target is None and node is not None and node.node_type == "station" and node.capacity is not None:
            options = [(cost, neighbor) for neighbor, cost, edge in adjacency.get(origin, ())
                       if state.get_node(neighbor).node_type != "station"
                       and _legal_move(state, unit, neighbor, edge)]
            if options:
                return min(options)[1]
        return None

    choices, ideal = [], None
    for neighbor, edge_cost, edge in adjacency.get(origin, ()):
        remaining = _distance(distances, neighbor, target)
        if not math.isfinite(remaining):
            continue
        base = edge_cost + remaining
        if ideal is None or (base, neighbor) < (ideal[0], ideal[1]):
            ideal = (base, neighbor, edge)
        if not _legal_move(state, unit, neighbor, edge):
            continue
        node = state.get_node(neighbor)
        dock_penalty = 3 if node.node_type == "station" and neighbor != target else 0
        reverse_penalty = 2 if _previous_node.get(unit.id) == neighbor else 0
        choices.append((base + dock_penalty + reverse_penalty, base, neighbor))

    if not choices or ideal is None:
        return None
    choices.sort()
    _, base, neighbor = choices[0]
    ideal_cost, ideal_neighbor, ideal_edge = ideal
    if ideal_neighbor != neighbor and not _legal_move(state, unit, ideal_neighbor, ideal_edge):
        wait = _blocked_wait(state, origin, ideal_neighbor, ideal_edge)
        if base >= ideal_cost + wait:
            return None
    elif base > ideal_cost + 3:
        return None
    return neighbor


def _edge_key(edge):
    if edge.bidirectional:
        return (min(edge.from_node, edge.to_node),
                max(edge.from_node, edge.to_node), True)
    return (edge.from_node, edge.to_node, False)


def _reserve(resources, kind, resource_id, first, last, change=1):
    for time in range(first, min(last, 80) + 1):
        key = (kind, resource_id, time)
        resources[key] += change


def _observed_resources(state, excluded_unit_id):
    """Reserve resources other robots are currently known to occupy."""
    resources = defaultdict(int)
    for unit in state.drive_units:
        if unit.id == excluded_unit_id:
            continue
        if unit.in_transit:
            edge = state.get_edge(unit.current_node, unit.transit_destination)
            remaining = _travel_time(unit.transit_remaining_time)
            _reserve(resources, 'edge', _edge_key(edge),
                     0, remaining - 1)
            _reserve(resources, 'node', unit.transit_destination,
                     0, remaining + 1)
        else:
            # A currently idle robot can depart during this step. Occupancy
            # beyond it is handled by the coordinated planner after assignment.
            _reserve(resources, 'node', unit.current_node, 0, 0)
    return resources


def _feasible_pair_cost(state, source, bundle, position, delay,
                        adjacency, distances, nodes, resources):
    """Estimate the score of actually collecting and delivering this bundle."""
    if not bundle:
        return None
    pickup_route = _time_path(source, position, delay, adjacency,
                              distances, resources, nodes)
    if pickup_route is None:
        return None
    pickup_time = pickup_route[-1][1][0] if pickup_route else delay
    stations = sorted({pod.destination_station for pod in bundle})
    if len(stations) <= 3:
        orders = permutations(stations)
    else:
        # Keep candidate evaluation bounded for hidden cases with many docks.
        remaining, current, order = set(stations), source, []
        while remaining:
            chosen = min(remaining,
                         key=lambda station: (_distance(distances, current, station), station))
            order.append(chosen)
            remaining.remove(chosen)
            current = chosen
        orders = [order]

    best_gain = None
    for order in orders:
        current, time = source, pickup_time + 1
        gain = 0.0
        for station in order:
            route = _time_path(station, current, time, adjacency,
                               distances, resources, nodes)
            if route is None:
                break
            arrival = route[-1][1][0] if route else time
            for pod in bundle:
                if pod.destination_station == station:
                    duration = max(0, state.current_time_step + arrival - 1 - pod.entry_time)
                    gain += math.exp(-duration / 50)
            current, time = station, arrival + 1
        else:
            best_gain = gain if best_gain is None else max(best_gain, gain)
    return -best_gain if best_gain is not None else None


def _time_path(target, start_node, start_time, adjacency, distances, resources, nodes):
    """Find an earliest-arrival route through a short reservation horizon."""
    if not math.isfinite(_distance(distances, start_node, target)):
        return None
    horizon = min(80, max(30, start_time + _distance(distances, start_node, target) + 16))
    start = (start_time, start_node)
    queue = [(start_time + _distance(distances, start_node, target),
              start_time, start_node)]
    parent = {start: None}

    while queue:
        _, time, node_id = heapq.heappop(queue)
        if node_id == target:
            route = []
            cursor = (time, node_id)
            while parent[cursor] is not None:
                previous, edge = parent[cursor]
                route.append((previous, cursor, edge))
                cursor = previous
            route.reverse()
            return route
        if time >= horizon:
            continue

        for neighbor, duration, edge in adjacency.get(node_id, ()):
            arrival = time + duration
            if arrival > horizon:
                continue
            if edge.capacity is not None and any(
                resources[('edge', _edge_key(edge), t)] >= edge.capacity
                for t in range(time, arrival)):
                continue
            destination = nodes[neighbor]
            if destination.capacity is not None and any(
                resources[('node', neighbor, t)] >= destination.capacity
                for t in range(time, arrival + 1)):
                continue
            next_state = (arrival, neighbor)
            if next_state not in parent:
                parent[next_state] = ((time, node_id), edge)
                heapq.heappush(queue, (arrival + _distance(distances, neighbor, target),
                                       arrival, neighbor))

        next_state = (time + 1, node_id)
        current = nodes[node_id]
        if (next_state not in parent and
            (current.capacity is None or
             resources[('node', node_id, time + 1)] < current.capacity)):
            parent[next_state] = ((time, node_id), None)
            heapq.heappush(queue, (time + 1 + _distance(distances, node_id, target),
                                   time + 1, node_id))
    return None


def _reserve_route(resources, unit, route, start_node, start_time):
    for (departure, origin), (arrival, destination), edge in route:
        if edge is None:
            _reserve(resources, 'node', origin, departure, arrival)
        else:
            _reserve(resources, 'node', origin, departure, departure)
            _reserve(resources, 'edge', _edge_key(edge), departure, arrival - 1)
            _reserve(resources, 'node', destination, departure, arrival)
    if route:
        (arrival, destination) = route[-1][1]
    else:
        arrival, destination = start_time, start_node
    # The engine replans at the next step after pickup or delivery; do not
    # pretend the unit parks on a task endpoint for the entire horizon.
    _reserve(resources, 'node', destination, arrival, arrival + 1)


def _plan_in_order(state, targets, adjacency, distances, order):
    """Build one reservation plan for a supplied robot priority order."""
    resources = defaultdict(int)
    base = {}
    nodes = {node.id: node for node in state.nodes}
    for unit in state.drive_units:
        entries = []
        if unit.in_transit:
            edge = state.get_edge(unit.current_node, unit.transit_destination)
            remaining = _travel_time(unit.transit_remaining_time)
            entries.append(('edge', _edge_key(edge), 0, remaining - 1))
            entries.append(('node', unit.transit_destination, 0,
                            remaining))
        else:
            entries.append(('node', unit.current_node, 0, 0))
        base[unit.id] = entries
        for kind, resource_id, first, last in entries:
            _reserve(resources, kind, resource_id, first, last)

    plans = {}
    arrivals = {}
    for unit in order:
        for kind, resource_id, first, last in base[unit.id]:
            _reserve(resources, kind, resource_id, first, last, -1)
        start_node, start_time = _position_and_delay(unit)
        target = targets.get(unit.id)
        if target is None and not unit.in_transit:
            node = nodes[start_node]
            if node.node_type == 'station' and node.capacity is not None:
                exits = [(cost, neighbor) for neighbor, cost, edge in adjacency[start_node]
                         if nodes[neighbor].node_type != 'station']
                if exits:
                    target = min(exits)[1]
        if target is None or target == start_node:
            if target is not None:
                arrivals[unit.id] = (start_time, target)
            if unit.in_transit:
                for kind, resource_id, first, last in base[unit.id]:
                    _reserve(resources, kind, resource_id, first, last)
            _reserve(resources, 'node', start_node, start_time, 80)
            plans[unit.id] = None
            continue

        route = _time_path(target, start_node, start_time,
                           adjacency, distances, resources, nodes)
        if route is None:
            for kind, resource_id, first, last in base[unit.id]:
                _reserve(resources, kind, resource_id, first, last)
            plans[unit.id] = _NO_PATH
            continue
        for kind, resource_id, first, last in base[unit.id]:
            if unit.in_transit:
                _reserve(resources, kind, resource_id, first, last)
        _reserve_route(resources, unit, route, start_node, start_time)
        arrivals[unit.id] = (route[-1][1][0] if route else start_time, target)
        first = route[0] if route else None
        plans[unit.id] = (first[1][1] if first is not None and
                          first[0][0] == 0 and first[2] is not None else None)
    return plans, arrivals


def _plan_value(state, arrivals, distances):
    """Estimate delivery reward after each robot's reserved first goal."""
    pod_by_id = {pod.id: pod for pod in state.active_pods}
    waiting = defaultdict(list)
    for pod in state.active_pods:
        if pod.carried_by is None and pod.current_node is not None:
            waiting[pod.current_node].append(pod)
    for pods in waiting.values():
        pods.sort(key=lambda pod: (pod.entry_time, pod.id))
    value = 0.0
    units = {unit.id: unit for unit in state.drive_units}
    for unit_id in sorted(arrivals, key=lambda uid: (arrivals[uid][0], uid)):
        arrival, target = arrivals[unit_id]
        unit = units[unit_id]
        pods = [pod_by_id[pod_id] for pod_id in unit.carrying
                if pod_id in pod_by_id]
        space = max(0, unit.capacity - len(pods))
        pods += waiting[target][:space]
        del waiting[target][:space]
        plan = _delivery_plan_score(state, pods, distances, target, arrival)
        if plan is not None:
            value += plan[0]
    return value


def _plan_all_moves(state, targets, adjacency, distances):
    """Repair a small reservation plan with up to three priority swaps."""
    order = sorted(
        state.drive_units,
        key=lambda unit: (not bool(unit.carrying),
                          _distance(distances, _position_and_delay(unit)[0],
                                    targets.get(unit.id)),
                          unit.id))
    best_plans, arrivals = _plan_in_order(state, targets, adjacency, distances, order)
    if not (2 <= len(order) <= 6 and len(targets) >= 2):
        return best_plans
    best_value = _plan_value(state, arrivals, distances)
    for index in range(min(3, len(order) - 1)):
        alternative = order[:]
        alternative[index], alternative[index + 1] = alternative[index + 1], alternative[index]
        plans, arrivals = _plan_in_order(state, targets, adjacency, distances, alternative)
        value = _plan_value(state, arrivals, distances)
        if value > best_value + 1e-6:
            best_value, best_plans = value, plans
    return best_plans


def _evacuate_idle_blockers(state, targets, adjacency, distances):
    """Move an unassigned robot off a full node needed by another robot."""
    # Expose dock exits as goals so a robot blocking that exit can yield too.
    for unit in state.drive_units:
        if unit.in_transit or unit.carrying or unit.id in targets:
            continue
        node = state.get_node(unit.current_node)
        if node.node_type != 'station' or node.capacity is None:
            continue
        exits = [(not _legal_move(state, unit, neighbor, edge), duration, neighbor)
                 for neighbor, duration, edge in adjacency.get(unit.current_node, ())
                 if state.get_node(neighbor).node_type != 'station']
        if exits:
            targets[unit.id] = min(exits)[2]
    reserved_goals = set(targets.values())
    pickup_nodes = {pod.current_node for pod in state.active_pods
                    if pod.carried_by is None and pod.current_node is not None}
    for unit in state.drive_units:
        if unit.in_transit or unit.carrying or unit.id in targets:
            continue
        origin = unit.current_node
        node = state.get_node(origin)
        if (node.capacity is None or node.node_type == 'station' or
                state.node_occupancy(origin) < node.capacity):
            continue
        obstructs = False
        for other in state.drive_units:
            target = targets.get(other.id)
            if other.id == unit.id or other.in_transit or target is None:
                continue
            edge = state.get_edge(other.current_node, origin)
            if edge is None:
                continue
            via = _travel_time(edge.weight) + _distance(distances, origin, target)
            if math.isfinite(via) and via == _distance(distances, other.current_node, target):
                obstructs = True
                break
        if not obstructs:
            continue
        options = []
        for neighbor, duration, edge in adjacency.get(origin, ()):
            destination = state.get_node(neighbor)
            if (destination.capacity is None and destination.node_type != 'station'
                    and neighbor not in reserved_goals and neighbor not in pickup_nodes
                    and math.isfinite(_distance(distances, neighbor, origin))
                    and _legal_move(state, unit, neighbor, edge)):
                options.append((destination.node_type != 'travel', duration, neighbor))
        if options:
            target = min(options)[2]
            targets[unit.id] = target
            reserved_goals.add(target)


def _position_idle_units(state, targets, distances):
    """Return unassigned robots toward storage before new pods arrive."""
    storages = [node.id for node in state.nodes
                if node.node_type == 'storage' and node.capacity is None]
    if not storages:
        return
    coverage = defaultdict(int)
    available = []
    for unit in state.drive_units:
        if unit.carrying:
            continue
        if unit.id in targets:
            coverage[targets[unit.id]] += 1
            continue
        position, delay = _position_and_delay(unit)
        if position in storages:
            targets[unit.id] = position
            coverage[position] += 1
        else:
            available.append((unit, position, delay))
    for unit, position, delay in sorted(available, key=lambda item: item[0].id):
        reachable = [source for source in storages
                     if math.isfinite(_distance(distances, position, source))]
        if reachable:
            source = min(reachable,
                         key=lambda source: (coverage[source],
                                             _distance(distances, position, source), source))
            targets[unit.id] = source
            coverage[source] += 1


def drive_unit_next_move(drive_unit_id: int, state: GraphState) -> Optional[int]:
    """Return an adjacent node ID for an idle unit, or None to wait."""
    unit = state.get_drive_unit(drive_unit_id)
    if unit is None or unit.in_transit:
        return None
    global _plan_time, _plan_moves, _last_polled_unit
    # Engine calls are in ascending ID order. Repeating that sequence at the
    # same time can be a new one-step game in a reused Python process.
    if state.current_time_step == _last_time and drive_unit_id <= _last_polled_unit:
        _plan_time, _plan_moves = -1, {}
        _previous_node.clear()
    _last_polled_unit = drive_unit_id
    adjacency, distances = _prepare_graph(state)
    targets = _assign_targets(state, distances, adjacency)
    _position_idle_units(state, targets, distances)
    _evacuate_idle_blockers(state, targets, adjacency, distances)
    if any(edge.capacity is not None for edge in state.edges):
        if _plan_time != state.current_time_step:
            _plan_moves = _plan_all_moves(state, targets, adjacency, distances)
            _plan_time = state.current_time_step
        move = _plan_moves.get(drive_unit_id, _NO_PATH)
        if move is not _NO_PATH and move is not None:
            edge = state.get_edge(unit.current_node, move)
            if edge is None or not _legal_move(state, unit, move, edge):
                move = _NO_PATH
        if move is _NO_PATH:
            move = _choose_move(unit, targets.get(drive_unit_id),
                                state, adjacency, distances)
    else:
        move = _choose_move(unit, targets.get(drive_unit_id),
                            state, adjacency, distances)
    if move is not None:
        _previous_node[drive_unit_id] = unit.current_node
    return move

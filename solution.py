"""Candidate solution for the assignment "Map, Data Association & Path Planning".

Run it with::

    python3 solution.py

The script executes the full pipeline step by step and stops at the first
step you have not implemented yet, with a reminder of what is missing.
Implement the three functions below (``associate``, ``correct_map``,
``find_path``), keeping their names and signatures unchanged.
"""

import math
from typing import List, Tuple

from map_generation import Map, Point, generate_map, generate_observations
from visualization import (
    visualize_association_n_correction,
    visualize_map_n_observations,
    visualize_path,
)

# ---------------------------------------------------------------------------
# Parameters of the run.
#
# Feel free to experiment with them, but note that your solution will also be
# evaluated with different values (especially with different random seeds).
# ---------------------------------------------------------------------------
NUM_POINTS = 30
X_RANGE = 20.0
Y_RANGE = 20.0
MAP_SEED = 24

NOISE_STD = 0.55
OBSERVATION_RATE = 0.8
OBSERVATIONS_SEED = 7


# ---------------------------------------------------------------------------
# Provided helper
# ---------------------------------------------------------------------------

def path_length(points: List[Point], path: List[int]) -> float:
    """Total Euclidean length of ``path`` over ``points``.

    Useful to measure how short your path is, e.g. when going for Bonus 1.
    The length is the sum of the Euclidean distances between the consecutive
    nodes of the path.
    """
    return sum(math.dist(points[a], points[b]) for a, b in zip(path, path[1:]))


# ---------------------------------------------------------------------------
# Step 3 -- Data association (yours to implement)
# ---------------------------------------------------------------------------

def associate(map_points: Map, observations: List[Point]) -> List[Tuple[int, int]]:
    """Associate every observation with the map point it was generated from.

    Given the map and the observations (noisy, incomplete and shuffled),
    recover the observation-to-map-point correspondence.

    Returns:
        The association couples as a list of ``(map_index, observation_index)``
        tuples, e.g. ``[(0, 12), (3, 5)]`` means that observation 12 was
        generated from map point 0, and observation 5 from map point 3.
    """


    """
    Creo una lista con ogni copia observation, map points e la loro distanza
    seleziono le copie con minor distanza saltando quelle che contengono punti di coppie gia selezionate
    """

    pairs = []
    for x, o in enumerate(observations):
        for y, m in enumerate(map_points): 
            pairs.append((math.dist(o, m), y, x))

    pairs.sort()

    used_map = []
    used_obs = [] 
    out = []

    for p in pairs:
        if p[1] in used_map or p[2] in used_obs:
            continue 
        out.append((p[1], p[2]))
        used_map.append(p[1])
        used_obs.append(p[2])
        if len(out) == len(observations): # se ho trovato una coppia per ogni obs mi fermo
            break
        
    return out

    #raise NotImplementedError("Step 3 (data association) is yours to implement")


# ---------------------------------------------------------------------------
# Step 4 -- Map correction (yours to implement)
# ---------------------------------------------------------------------------

def correct_map(
    map_points: Map,
    observations: List[Point],
    associations: List[Tuple[int, int]],
) -> Map:
    """Correct the map, given the data association.

    For every association couple ``(i, j)``, the corrected position of map
    point ``i`` is the average (midpoint) of ``map_points[i]`` and
    ``observations[j]``. Map points without an associated observation keep
    their original position.

    Returns:
        The corrected map: a list of ``(x, y)`` tuples with the same length
        and the same order as ``map_points``.
    """    

    """
    Copio map_points e per ogni associazione sovrascrivo il punto con la media
    """

    corrected = [(p[0], p[1]) for p in map_points]

    for a in associations:
        mx, my = map_points[a[0]]
        ox, oy = observations[a[1]]
        corrected[a[0]] = ((mx + ox) / 2, (my + oy) / 2)

    return corrected

    #raise NotImplementedError("Step 4 (map correction) is yours to implement")


# ---------------------------------------------------------------------------
# Step 5 -- Path finding (yours to implement)
# ---------------------------------------------------------------------------

def find_path(corrected_map: Map) -> List[int]:
    """Find a path that visits every node of the corrected map exactly once.

    Return the path as a list of node indices in visit order, e.g.
    ``[4, 0, 11, 2, ...]``. A path that includes each node exactly once is
    usually called a *Hamiltonian path*: any permutation of the node indices
    is a formally valid answer, but:

    * **Bonus 1 (+1 point)**: use a heuristic or an algorithm (e.g. nearest
      neighbour, greedy insertion, 2-opt, ...) so that the total length of
      your path is clearly shorter than the worst possible path. You can
      measure it with :func:`path_length`.

    * **Bonus 2 (+1 point)**: no arrow of the path goes over a previous
      arrow (arrows that only touch at a shared node do not count as
      crossings). You can check it visually with ``visualize_path``.
    """

    """
    uso nearest neighbour come base e poi 2-opt per efficentarlo
    """



    pts = list(corrected_map)
    n = len(pts)
    if n <= 2:
        return list(range(n))

    def d(a: int, b: int) -> float:# get distance from index
        return math.dist(pts[a], pts[b])

    # nearest neighbour partendo dal nodo 0, 0 scelto arbitrariamente
    unvisited = set(range(1, n))
    path = [0]
    while unvisited:
        last = path[-1]
        nxt = min(unvisited, key=lambda k: d(last, k))
        path.append(nxt)
        unvisited.remove(nxt)

    # 2-opt
    eps = 1e-12
    improved = True
    while improved:
        improved = False
        for i in range(n - 1):
            for j in range(i + 1, n):
                
                before = d(path[i - 1], path[i]) if i > 0 else 0.0
                after = d(path[j], path[j + 1]) if j < n - 1 else 0.0
                new_before = d(path[i - 1], path[j]) if i > 0 else 0.0
                new_after = d(path[i], path[j + 1]) if j < n - 1 else 0.0

                if new_before + new_after < before + after - eps:
                    path[i:j + 1] = reversed(path[i:j + 1])
                    improved = True

    return path



    #raise NotImplementedError("Step 5 (path finding) is yours to implement")


# ---------------------------------------------------------------------------
# Pipeline
# ---------------------------------------------------------------------------

def main() -> None:
    # Step 1 -- Generate the map (provided) ---------------------------------
    map_points = generate_map(
        num_points=NUM_POINTS,
        x_range=X_RANGE,
        y_range=Y_RANGE,
        seed=MAP_SEED,
    )

    # Step 2 -- Generate the observations (provided) ------------------------
    observations = generate_observations(
        map_points,
        noise_std=NOISE_STD,
        observation_rate=OBSERVATION_RATE,
        seed=OBSERVATIONS_SEED,
    )
    visualize_map_n_observations(map_points, observations, X_RANGE, Y_RANGE)

    # Step 3 -- Data association (yours) -------------------------------------
    try:
        associations = associate(map_points, observations)
    except NotImplementedError:
        print("Step 3 is not implemented yet: fill in associate() in solution.py.")
        return

    # Step 4 -- Map correction (yours) ---------------------------------------
    try:
        corrected_map = correct_map(map_points, observations, associations)
    except NotImplementedError:
        print("Step 4 is not implemented yet: fill in correct_map() in solution.py.")
        return
    visualize_association_n_correction(
        map_points,
        observations,
        corrected_map,
        associations,
        X_RANGE,
        Y_RANGE,
    )

    # Step 5 -- Path finding (yours) -----------------------------------------
    try:
        path = find_path(corrected_map)
    except NotImplementedError:
        print("Step 5 is not implemented yet: fill in find_path() in solution.py.")
        return
    visualize_path(corrected_map, path)

    print(
        f"Pipeline complete: {len(associations)} associations over "
        f"{len(observations)} observations, path over {len(corrected_map)} "
        f"nodes with total length {path_length(corrected_map, path):.2f}."
    )


if __name__ == "__main__":
    main()

# Path Planning

algorithm3 is the "3D" variant of all search algorithms

view3 is the "3D" variant for drawing the graph

Use RRTStarV2, it is the newest variation


algrotihm run-down

1 search()

    Add node x_init to graph
    count = 0
    while 'critera':
        r = shrinking_ball_radius
        if rrt obstacles != graph obstacles:
            update obstacles with rrt obstacles
            propagate descendents
        x_rand = local_random_sample
        x_nearest = nearest_node
        x_new = steer
        X_near = extend
        if x_new is an orphan:
            add x_new to list of orphans
        if x_new is in the graph:
            rewire_neighbors
            reduce_inconsistency
        add 1 to count

2 extend(x_new, x_nearest, r)

    X_near = near
    Add node x_new to graph
    find_parent
    3 find_parent(x_new, x_min, x_near)
    if collision_free:
        minimum cost
        for x_near in X_near"
            if collision_free and cost < minimum cost:
                x_min = x_near
                minimum cost = cost                
        Add edge (x_min, x_new) to graph

4 rewire neighbors(x_new, X_near)

    for x_near in X_near:
        if collision free and cost of new < cost of near:
            set parent to parent of x_near
            if no parent:
                continue
            remove edge (x_parent, x_near) from graph
            add edge (x_new, x_near) to graph

5 propogate_descendents()

    for n in graph nodes:
        if n in obstacle:
            parent = parent(n)
            if parent exists:            
                remove edge (parent, n) from graph

6 reduce_inconsistency()

    if no orphans:
        return
    leaves = []
    for n in graph nodes:
        if n is a leaf and not a orphan:
            Add n to list, leaves
    for n in orphans:
        if n is not in an obstacle:
            nearest = brute_force(n, leaves)
            if (nearest, n) collision free:
                Add edge (nearest, n) to graph
                Add n to leaves
                Remove n from orphans

7 update_obstacles(obstacles)

    for obstacle in obstacles:
        if obstacle in graph obstacles or list of obstacles:
            for obstacle in graph obstacles:
                remove obstacles from graph obstacles
    if obstacles not in graph obstacles and obstacles:
        for obstacle in obstacles:
            Add obstacle to graph obstacles

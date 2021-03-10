import matplotlib.pyplot as plt


def draw(graph, start, goal):
    """Plots the graph."""
    step = int((graph.span[0][1] - graph.span[0][0]) / 40)
    plt.figure(figsize=[9.6, 7.2])
    plt.xlim(graph.span[0][0], graph.span[0][1])
    plt.ylim(graph.span[1][0], graph.span[1][1])
    plt.xticks(range(graph.span[0][0], graph.span[0][1]+step, step))
    plt.yticks(range(graph.span[1][0], graph.span[1][1]+step, step))
    plt.grid(linestyle='-.')
    # for e in graph._edge:
    #     x = ([e[0][0], e[1][0]])
    #     y = ([e[0][1], e[1][1]])
    #     plt.scatter(x, y, s=2, c='#7FFF00', zorder=2)
    #     plt.plot(x, y,  c='#00008B', lw='0.5', zorder=1)
    # for p in path:
    #     x, y, z = p
    #     plt.scatter(x, y, s=5, c='#0000FF', zorder=2)
    for o in graph.obstacles:
        if graph.obstacles == []:
            pass
        else: 
            x, y = list(o.exterior.xy)
            plt.plot(x, y, c='#FF0000', zorder=1)

    # plt.scatter(bogey[0], bogey[1], s=20, c='#FF0000', marker='.', zorder=2)
    # plt.scatter(bogey[0], bogey[1], s=2800, c='#FFFFFF', marker='o', zorder=1)
    # plt.scatter(bogey[0], bogey[1], s=3000, c='#FF0000', marker='o', zorder=0)
    plt.scatter(start[0], start[1], s=50, c='#000000', marker="s", zorder=0)
    plt.scatter(start[0], start[1], s=40, c='#FFFF00', marker="s", zorder=0)
    plt.scatter(goal[0], goal[1], s=100, c='#FF00FF', marker="s", zorder=0)
    print('Mapping.')
    plt.show()

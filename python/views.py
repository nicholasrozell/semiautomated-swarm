import matplotlib.pyplot as plt


def draw(graph, s, g, path=None, leaves=None):
    """
    Plots the graph.
    """
    plt.figure(figsize=[9, 8])
    plt.xlim(graph.dims[0][0], graph.dims[0][1])
    plt.ylim(graph.dims[1][0], graph.dims[1][1])
    plt.grid(linestyle='-.')
    for n in graph._node:
        x, y, z = n
        plt.scatter(x, y, s=2, c='#7FFF00', zorder=2)
    for e in graph._edge:
        x = ([e[0][0], e[1][0]])
        y = ([e[0][1], e[1][1]])
        plt.plot(x, y, c='#00008B', lw='0.5', zorder=1)
    if path is not None:
        for i in range(len(path)):
            for j in range(len(path[i])-1):
                x = ([path[i][j][0], path[i][j+1][0]])
                y = ([path[i][j][1], path[i][j+1][1]])
                plt.plot(x, y, c='#FAA0A0', lw='0.5', zorder=2)
                if j == 0:
                    plt.plot(x, y, c='#FF0000', lw='1', zorder=3)
    if leaves is not None:
        for leaf in leaves:
            x, y, z = leaf
            plt.scatter(x, y, s=2, c='#008000', zorder=2)
    # if graph.obstacles != []:
    #     for o in graph.obstacles:
    #         x, y = list(o.exterior.xy)
    #         plt.plot(x, y, c='#000000', zorder=1)
    plt.scatter(s[0], s[1], s=50, c='#000000', marker='s', zorder=0)
    plt.scatter(s[0], s[1], s=40, c='#FFFF00', marker='s', zorder=0)
    plt.scatter(g[0], g[1], s=100, c='#FF00FF', marker='s', zorder=0)

    plt.show()

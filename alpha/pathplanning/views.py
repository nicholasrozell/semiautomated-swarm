import matplotlib.pyplot as plt


def draw(graph, s, g, path=[]):
    """
    Plots the graph.
    """
    step = int((graph.span[0][1] - graph.span[0][0]))
    plt.figure(figsize=[8, 8])
    plt.xlim(graph.span[0][0], graph.span[0][1])
    plt.ylim(graph.span[1][0], graph.span[1][1])
    plt.grid(linestyle='-.')
    for e in graph._edge:
        x = ([e[0][0], e[1][0]])
        y = ([e[0][1], e[1][1]])
        plt.scatter(x, y, s=2, c='#7FFF00', zorder=2)
        plt.plot(x, y, c='#00008B', lw='0.5', zorder=1)
    if path != []:
        for p in path:
            x, y = p
            plt.scatter(x, y, s=5, c='#0000FF', zorder=2)
    if graph.obstacles != []:
        for o in graph.obstacles:
            x, y = list(o.exterior.xy)
            plt.plot(x, y, c='#FF0000', zorder=1)
    plt.scatter(s[0], s[1], s=50, c='#000000', marker='s', zorder=0)
    plt.scatter(s[0], s[1], s=40, c='#FFFF00', marker='s', zorder=0)
    plt.scatter(g[0], g[1], s=100, c='#FF00FF', marker='s', zorder=0)
    plt.show()

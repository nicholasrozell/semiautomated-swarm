import matplotlib.pyplot as plt


def draw(graph, s, g, path=None, enodes=[], leaves=[]):
    """
    Plots the graph.
    """
    plt.figure(figsize=[10, 10])
    plt.xlim(graph.span[0][0], graph.span[0][1])
    plt.ylim(graph.span[1][0], graph.span[1][1])
    plt.grid(linestyle='-.')
    for n in graph._node:
        x, y, z = n
        plt.scatter(x, y, s=2, c='#7FFF00', zorder=2)
    for e in graph._edge:
        x = ([e[0][0], e[1][0]])
        y = ([e[0][1], e[1][1]])
        plt.plot(x, y, c='#00008B', lw='0.5', zorder=1)
    if path != None:
        for p in path:
            x, y, z = p
            plt.scatter(x, y, s=5, c='#0000FF', zorder=2)
    if enodes != []:
        # for i in range(len(enodes)):
            # xl, yl, zl = leaves[i]
        xe, ye, ze = enodes
            # plt.plot((xl, xe), (yl, ye), c='#FF0000', lw=0.5, zorder=2)
            # plt.scatter(xl, yl, s=5, c='#FF0000', zorder=2)
        plt.scatter(xe, ye, s=5, c='#FF0000', zorder=2)
    if graph.obstacles != []:
        for o in graph.obstacles:
            x, y = list(o.exterior.xy)
            plt.plot(x, y, c='#FF0000', zorder=1)
    plt.scatter(s[0], s[1], s=50, c='#000000', marker='s', zorder=0)
    plt.scatter(s[0], s[1], s=40, c='#FFFF00', marker='s', zorder=0)
    plt.scatter(g[0], g[1], s=100, c='#FF00FF', marker='s', zorder=0)
    plt.show()
    # plt.show(block=False)
    # plt.pause(1)
    # plt.close()

def draw2(graph, s, g, path=None, position=None):
    """
    Plots the graph.
    """
    plt.figure(figsize=[9, 9])
    plt.xlim(graph.span[0][0], graph.span[0][1])
    plt.ylim(graph.span[1][0], graph.span[1][1])
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
                plt.plot(x, y, c='#006400', lw='0.5', zorder=2)
                if j == 0:
                    plt.scatter(x, y, s=5, c='#FF0000', zorder=3)
    if position != None:
        for i in range(len(position)-1):
            x = ([position[i][0], position[i+1][0]])
            y = ([position[i][1], position[i+1][1]])
            plt.plot(x, y, lw=1.5, c='#9400D3', zorder=2)
    if graph.obstacles != []:
        for o in graph.obstacles:
            x, y = list(o.exterior.xy)
            plt.plot(x, y, c='#000000', zorder=1)
    plt.scatter(s[0], s[1], s=50, c='#000000', marker='s', zorder=0)
    plt.scatter(s[0], s[1], s=40, c='#FFFF00', marker='s', zorder=0)
    plt.scatter(g[0], g[1], s=100, c='#FF00FF', marker='s', zorder=0)
    plt.show()

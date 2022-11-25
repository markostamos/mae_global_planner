import matplotlib.pyplot as plt


def plotTargets(points, axes):
    axes.scatter(points[0], points[1], c="r")


def plotGlobalPlan(gplan, axes):
    colors = ["green", "blue", "red", "magenta", "m", "k"]
    for n, plan in enumerate(gplan):
        axes.plot(plan[0], plan[1], c=colors[n % len(colors)])


def plotClusters(clusters, axes):
    colors = ["green", "blue", "red", "magenta", "m", "k"]
    for n, cluster in enumerate(clusters):
        # random color
        axes.scatter(cluster[0], cluster[1], c=colors[n % len(colors)])


def plotAgents(points, axes):
    axes.scatter(points[0], points[1], c="black", marker="D", s=100)


def readCSV(filename):
    xs = []
    ys = []
    zs = []
    with open(filename, "r") as f:
        for line in f:
            xs.append(float(line.split(",")[0]))
            ys.append(float(line.split(",")[1]))
            zs.append(float(line.split(",")[2]))
    return [xs, ys, zs]


def readMultiCSV(filename):
    clusters = []
    xs = []
    ys = []
    zs = []
    with open(filename, "r") as f:
        for line in f:
            if line.strip():
                xs.append(float(line.split(",")[0]))
                ys.append(float(line.split(",")[1]))
                zs.append(float(line.split(",")[2]))
            else:
                clusters.append([xs, ys, zs])
                xs = []
                ys = []
                zs = []
    return clusters


if __name__ == "__main__":
    targets = readCSV("build/targets.csv")
    agents = readCSV("build/agents.csv")
    clusters = readMultiCSV("build/clusters.csv")
    gplan = readMultiCSV("build/plan.csv")
    iplan = readMultiCSV("build/initial_plan.csv")

    f = plt.figure()
    f, axes = plt.subplots(nrows=2, ncols=2, sharex=True, sharey=True)

    axes[0, 0].set_title("Random Initialization")
    plotTargets(targets, axes[0][0])
    plotAgents(agents, axes[0][0])
    axes[0, 0].legend(["Targets", "Agents"])

    axes[0, 1].set_title("K-Means Clustering")
    plotClusters(clusters, axes[0][1])
    plotAgents(agents, axes[0][1])

    axes[1, 0].set_title("Initial Random Plan")
    plotClusters(clusters, axes[1][0])
    plotAgents(agents, axes[1][0])
    plotGlobalPlan(iplan, axes[1][0])

    axes[1, 1].set_title("Optimized Plan")
    plotClusters(clusters, axes[1][1])
    plotGlobalPlan(gplan, axes[1][1])
    plotAgents(agents, axes[1][1])
    plt.show()

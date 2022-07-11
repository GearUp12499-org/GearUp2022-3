from unittest import result
import matplotlib.pyplot as plt

CLICKS_PER_SECOND = 20000
REAL_CLICKS_PER_SECOND = 10000
ACC = 0.2
CAP = 1

RUNTIME = 10
TARGET = 1000

def process(distance_to_target: int, current_power: float, dt: float):
    global CAP, ACC, CLICKS_PER_SECOND
    if distance_to_target < 0:
        return 0, 0, 0
    eta = float('inf') if current_power == 0 else distance_to_target / (CLICKS_PER_SECOND * current_power)
    target_power = min(CAP, ACC * eta)
    if target_power != current_power:
        resulting_power = current_power + (dt * ACC if target_power > current_power else -dt * ACC)
    else:
        resulting_power = current_power
    return resulting_power, target_power, eta


def main():
    global TARGET, RUNTIME, CAP, ACC, CLICKS_PER_SECOND, REAL_CLICKS_PER_SECOND
    powers = []
    power = 0
    targets = []
    etas = []
    dists = []
    dist_travelled = 0
    for i in range(RUNTIME * 100):
        power, target, eta = process(TARGET - dist_travelled, power, 0.01)
        targets.append(target)
        etas.append(eta)
        dist_travelled += power * REAL_CLICKS_PER_SECOND * 0.01
        dists.append(dist_travelled)
        powers.append(power)
    
    fig, ax = plt.subplots()
    ax.plot(powers, label="Power")
    ax.plot(targets, label="Target Power")
    ax.legend()
    fig.show()
    fig, ax = plt.subplots()
    ax.plot(dists, label="Distance Travelled")
    ax.legend()
    fig.show()
    input(".")


if __name__ == "__main__":
    main()

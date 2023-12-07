import numpy as np

def read_file(fname):
    results = {
        "write acquire": [],
        "read acquire": [],
        "map actor queue size": []
    }
    with open(fname, "r") as f:
        for line in f:
            if "TimerFinished" in line:
                try:
                    title, time = line.split("|")[3].split(",")
                    time = time.split("=")[1].strip()
                    if "ms" in time:
                        time = float(time.split("ms")[0].strip())
                    elif "µs" in time:
                        time = float(time.split("µs")[0].strip()) / 1000
                    elif "ns" in time:
                        time = float(time.split("ns")[0].strip()) / 1000000
                    elif "s" in time:
                        time = float(time.split("s")[0].strip()) * 1000
                    else:
                        print("Unknown unit??", line)

                    if title in results:
                        results[title].append(time)
                    else:
                        results[title] = [time]
                except ValueError:
                    print(line)
            elif "LOCKS...Write acquire" in line:
                results["write acquire"].append(float(line.split("Write acquire:")[1].strip().split(" ")[0].strip()))
            elif "LOCKS...Read acquire" in line:
                results["read acquire"].append(float(line.split("Read acquire:")[1].strip().split(" ")[0].strip()))
            elif "map actor queue size" in line:
                results["map actor queue size"].append(float(line.split("map actor queue size:")[1].strip().split(" ")[0].strip()))




    for title in results.keys():
        avg = np.mean(results[title])
        std = np.std(results[title])
        results[title] = "{:.2f} += {:.2f}".format(avg, std)
    return results


results = read_file("../darvis/results/output.log")
for (title, time) in sorted(results.items()):
    print("{:<40} {:<10}".format(title, time))

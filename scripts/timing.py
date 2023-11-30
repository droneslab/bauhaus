import numpy as np

def read_file(fname):
    results = {
        "TRACKING FRONTEND": {},
        "TRACKING BACKEND": {},
        "LOCAL MAPPING": {},
        # "LOOP CLOSING": {},
        "INIT/LOOP CLOSING": {},
        "MAP": {},
        "LOCKS": {}
    }
    with open(fname, "r") as f:
        for line in f:
            if "TRACE" in line:
                try:
                    title, time = line.split("|")[3].split(":")
                    module, title = title.split("...")
                    time = int(time.split("ms")[0].strip())

                    if title in results[module]:
                        results[module][title].append(time)
                    else:
                        results[module][title] = [time]
                except ValueError:
                    print(line)

    for module in results.keys():
        for task in results[module].keys():
            avg = np.mean(results[module][task])
            std = np.std(results[module][task])
            results[module][task] = "{:.2f} += {:.2f}".format(avg, std)
    return results


results = read_file("../darvis/results/output.log")
for module in results.keys():
    print(module)
    print("=====")
    for task in results[module].keys():
        if task != "Total":
            print("{:<40} {:<10}".format(task, results[module][task]))
    try:
        print("{:<40} {:<10}".format("Total", results[module]["Total"]))
    except KeyError:
        continue
    print("\n")

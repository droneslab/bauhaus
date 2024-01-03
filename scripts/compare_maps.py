## Compares output of printing the entire map in ORB_SLAM3 and Darvis
## Print map by calling print_current_map() in darvis and PrintCurrentMap() in ORB_SLAM3

from pprint import pprint

def read_kf_pose(line):
    t, r = line.split("/")
    x, y, z = t.split("=")[1].strip().strip("[]").split(",")
    try:
        # Darvis
        i, j, k, w = r.split("=")[1].strip().strip("[]").split(",")
    except ValueError:
        # ORBSLAM3 doesn't print w
        i, j, k = r.split("=")[1].strip().strip("[]").split(",")
        w = 1

    return {
        "translation": [float(x), float(y), float(z)],
        "rotation": [float(i), float(j), float(k), float(w)]
    }

def read_mp_pose_orbslam(pose):
    x, y, z = pose.split(",")
    return [float(x), float(y), float(z)]

def read_mp_pose_darvis(line):
    x, y, z = line.split(",")
    x = x.split("[")[2]
    y = y.strip()
    z = z.split("]")[0].strip()
    return [float(x), float(y), float(z)]

def read(filename, read_mp_pose):
    data = {}
    curr_map_id = None
    current = None
    with open(filename, "r") as f:
        for line in f:
            if "PRINT MAP START" in line:
                _, curr_map_id = line.split(";")[0:2]
                curr_map_id = curr_map_id.strip()

                curr_map = {
                    "keyframes": {},
                    "mappoints": {}
                }
            elif "PRINT MAP KF" in line:
                _, id, pose, connections, mappoints = line.split(";")
                curr_map["keyframes"][int(id)] = {
                    "pose": read_kf_pose(pose),
                    "connections": set(map(lambda x: int(x), connections.strip().strip(",").split(","))),
                    "mappoints": set(map(lambda x: int(x), mappoints.strip().strip(",").split(",")))
                }
            elif "PRINT MAP MP" in line:
                _, id, pose, observations = line.split(";")
                curr_map["mappoints"][int(id)] = {
                    "pose": read_mp_pose(pose),
                    "observations": set(map(lambda x: int(x), observations.strip().strip(",").split(",")))
                }
            elif "PRINT MAP DONE" in line:
                data[curr_map_id] = curr_map
    return data

def is_pose_same(pose1, pose2, threshold=0.01):
    try:
        # For keyframes
        return all([
            abs(pose1["translation"][0] - pose2["translation"][0]) < threshold,
            abs(pose1["translation"][1] - pose2["translation"][1]) < threshold,
            abs(pose1["translation"][2] - pose2["translation"][2]) < threshold,
            abs(pose1["rotation"][0] - pose2["rotation"][0]) < threshold,
            abs(pose1["rotation"][1] - pose2["rotation"][1]) < threshold,
            abs(pose1["rotation"][2] - pose2["rotation"][2]) < threshold,
            abs(pose1["rotation"][3] - pose2["rotation"][3]) < threshold,
        ])
    except TypeError:
        # For mappoints
        return all([
            abs(pose1[0] - pose2[0]) < threshold,
            abs(pose1[1] - pose2[1]) < threshold,
            abs(pose1[2] - pose2[2]) < threshold,
        ])

def compare(d1, d2, threshold=0.01):
    for map_identifier in d1:
        print("========================")
        print("MAP: ", map_identifier)
        print("========================")

        if map_identifier not in d2:
            print("Map not in orbslam")
            continue

        map1 = d1[map_identifier]
        map2 = d2[map_identifier]

        for keyframe in map1["keyframes"]:
            if keyframe not in map2["keyframes"]:
                print("Added keyframe {}".format(keyframe))
                continue

            kf1 = map1["keyframes"][keyframe]
            kf2 = map2["keyframes"][keyframe]

            if not is_pose_same(kf1["pose"], kf2["pose"], threshold):
                print(
                    "Modified keyframe {} pose \n darvis: {} \n orbslam: {}"
                    .format(keyframe, kf1["pose"], kf2["pose"])
                )

            if kf1["connections"] != kf2["connections"]:
                print("Different connections for keyframe {} \n darvis: {} \n orbslam: {}"
                    .format(keyframe, kf1["connections"], kf2["connections"]))

            if kf1["mappoints"] != kf2["mappoints"]:
                print("Different mappoints for keyframe {} \n darvis: {} \n orbslam: {}"
                    .format(keyframe, kf1["mappoints"], kf2["mappoints"]))

        for keyframe in map2["keyframes"]:
            if keyframe not in map1["keyframes"]:
                print("Removed keyframe {}".format(keyframe))

        for mappoint in map1["mappoints"]:
            if mappoint not in map2["mappoints"]:
                print("Added mappoint {}".format(mappoint))
                continue

            mp1 = map1["mappoints"][mappoint]
            mp2 = map2["mappoints"][mappoint]

            if not is_pose_same(mp1["pose"], mp2["pose"], threshold):
                print("Modified mappoint {} pose \n darvis: {} \n orbslam: {}"
                    .format(mappoint, mp1["pose"], mp2["pose"]))
                continue

            if mp1["observations"] != mp2["observations"]:
                print("Different observations for mappoint {} \n darvis: {} \n orbslam: {}"
                    .format(mappoint, mp1["observations"], mp2["observations"]))
                continue

        for mappoint in map2["mappoints"]:
            if mappoint not in map1["mappoints"]:
                print("Removed mappoint {}".format(mappoint))

        print("\n\n")


orbslam_file = "/home/sofiya/ORB_SLAM3/save.txt"
darvis_file = "/home/sofiya/darvis/darvis/darvis/results/save.txt"

os = read(orbslam_file, read_mp_pose_orbslam)
darvis = read(darvis_file, read_mp_pose_darvis)

print("Comparing Darvis to ORB_SLAM3 \n")
compare(darvis, os, .001)

def txt_to_pcd(txt_file, pcd_file):
    with open(txt_file, 'r') as file:
        lines = file.readlines()

    points = []
    for line in lines[2:]:
        data = line.split()
        if len(data) >= 3:
            x, y, z = map(float, data[:3])
            points.append([x, y, z])

    header = [
        "# .PCD v0.7 - Point Cloud Data file format",
        "VERSION 0.7",
        "FIELDS x y z",
        "SIZE 4 4 4",
        "TYPE F F F",
        "COUNT 1 1 1",
        f"WIDTH {len(points)}",
        "HEIGHT 1",
        "VIEWPOINT 0 0 0 1 0 0 0",
        f"POINTS {len(points)}",
        "DATA ascii"
    ]

    with open(pcd_file, 'w') as file:
        file.write('\n'.join(header))
        file.write('\n')
        for point in points:
            file.write(f"{point[0]} {point[1]} {point[2]}\n")

    print(f"Finished! Convert {txt_file} to {pcd_file}")

if __name__ == '__main__':
    input = '/home/eason/data/data/runman.off'
    output = '/home/eason/data/data/runman.pcd'
    txt_to_pcd(input, output)
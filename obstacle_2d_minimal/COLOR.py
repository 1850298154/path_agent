# color
import csv
colors_hex = []
# 打开CSV文件
# with open('color.csv', 'r', encoding='utf-8') as csvfile:
with open('color.tsv.css', 'r', encoding='utf-8') as csvfile:
    reader = csv.reader(csvfile, delimiter='\t')  # 两个空格，表示连续空格

    # 跳过标题行
    next(reader)

    # 逐行读取数据
    for row in reader:
        # print(row)
        # english_name = row[0]
        # visual_color = row[1]
        hex_format = row[2]
        # rgb_format = row[3]
        colors_hex.append(hex_format)
        # # 提取RGB值
        # rgb_values = rgb_format.split(',')
        # red = int(rgb_values[0])
        # green = int(rgb_values[1])
        # blue = int(rgb_values[2])

        # # 输出结果
        # print("英文代码:", english_name)
        # print("形像颜色:", visual_color)
        # print("十六进制格式:", hex_format)
        # print("RGB格式:", rgb_format)
        # print("R值:", red)
        # print("G值:", green)
        # print("B值:", blue)
        # print()

if __name__ == "__main__":
    print(colors_hex)

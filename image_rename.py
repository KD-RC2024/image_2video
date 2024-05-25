import os
import re

def extract_number(filename):
    """
    从文件名中提取数字。

    Args:
        filename (str): 文件名。

    Returns:
        int: 提取出的数字，如果找不到数字则返回 0。
    """
    # 使用正则表达式匹配文件名中的数字
    match = re.search(r'\d+', filename)
    if match:
        return int(match.group())
    else:
        return 0

def get_jpg_filenames_sorted(directory):
    """
    获取指定目录中所有以.jpg为后缀的文件名列表，并按文件名中的数字排序。

    Args:
        directory (str): 要搜索的目录路径。

    Returns:
        list: 包含所有.jpg文件名的列表，并按文件名中的数字排序。
    """
    # 检查目录是否存在
    if not os.path.isdir(directory):
        print(f"目录 '{directory}' 不存在")
        return []

    jpg_filenames = []

    # 遍历目录中的所有文件
    for filename in os.listdir(directory):
        # 构建文件的完整路径
        filepath = os.path.join(directory, filename)
        # 检查路径是否是文件且以.jpg结尾
        if os.path.isfile(filepath) and filename.endswith('.jpg'):
            # 将文件名添加到列表中
            jpg_filenames.append(filename)

    # 按文件名中的数字进行排序
    jpg_filenames.sort(key=extract_number)

    return jpg_filenames


def get_txt_filenames_sorted(directory):
    """
    获取指定目录中所有以.txt为后缀的文件名列表，并按文件名中的数字排序。

    Args:
        directory (str): 要搜索的目录路径。

    Returns:
        list: 包含所有.txt文件名的列表，并按文件名中的数字排序。
    """
    # 检查目录是否存在
    if not os.path.isdir(directory):
        print(f"目录 '{directory}' 不存在")
        return []

    txt_filenames = []

    # 遍历目录中的所有文件
    for filename in os.listdir(directory):
        # 构建文件的完整路径
        filepath = os.path.join(directory, filename)
        # 检查路径是否是文件且以.jpg结尾
        if os.path.isfile(filepath) and filename.endswith('txt'):
            # 将文件名添加到列表中
            txt_filenames.append(filename)

    # 按文件名中的数字进行排序
    txt_filenames.sort(key=extract_number)

    return txt_filenames


def find_different_files(list1, list2):
    """
    找到两个文件名列表中不同的文件名，并将其存入一个新的列表中。

    Args:
        list1 (list): 第一个文件名列表。
        list2 (list): 第二个文件名列表。

    Returns:
        list: 包含不同文件名的列表。
    """
    # 将文件名列表转换为集合
    set1 = set(list1)
    set2 = set(list2)

    # 找到两个集合的差集
    different_files = set1.symmetric_difference(set2)

    return list(different_files)

def change_extension(filename_list):
    """
    将文件名列表中的.jpg后缀改成.txt后缀。

    Args:
        filename_list (list): 文件名列表。

    Returns:
        list: 包含修改后文件名的列表。
    """
    modified_filenames = []

    # 遍历文件名列表
    for filename in filename_list:
        # 将.jpg后缀改成.txt后缀，并添加到新列表中
        modified_filenames.append(filename.replace('.jpg', '.txt'))

    return modified_filenames


def delete_files_with_matching_names(folder_path, file_names):
    # 遍历文件夹中的所有文件
    for file_name in os.listdir(folder_path):
        # 获取文件的完整路径
        file_path = os.path.join(folder_path, file_name)
        # 如果文件是普通文件并且文件名（不含后缀）与文件名列表中的任何文件名相同，则删除文件
        if os.path.isfile(file_path):
            name_without_extension = os.path.splitext(file_name)[0]
            if name_without_extension in file_names:
                os.remove(file_path)
                print(f"Deleted: {file_name}")

# 要删除文件的文件夹路径
folder_path = "/path/to/folder"

# 文件名列表
file_names = ["file1", "file2", "file3"]

def delete_files(directory, filenames):
    """
    删除目录中与列表元素同名的文件。

    Args:
        directory (str): 要删除文件的目录路径。
        filenames (list): 要匹配删除的文件名列表。
    """
    # 遍历要删除的文件名列表
    for filename in filenames:
        # 构建文件的完整路径
        filepath = os.path.join(directory, filename)
        # 检查路径是否是文件
        if os.path.isfile(filepath):
            # 删除文件
            os.remove(filepath)
            print(f"已删除文件 '{filename}'")


def rename_and_sort_jpg_files(directory):
    """
    重新命名当前目录中的所有.jpg文件，使它们按照12345...的顺序进行命名，并进行排序。

    Args:
        directory (str): 当前目录路径。
    """
    # 获取当前目录中所有.jpg文件的文件名列表
    jpg_files = [filename for filename in os.listdir(directory) if filename.endswith('.jpg')]

    # 按文件名中的数字进行排序
    jpg_files.sort()

    # 逐个重命名.jpg文件
    for i, filename in enumerate(jpg_files, start=1):
        # 构建新文件名，使用当前索引作为数字部分
        new_filename = f"{i}.jpg"
        # 构建旧文件的完整路径
        old_filepath = os.path.join(directory, filename)
        # 构建新文件的完整路径
        new_filepath = os.path.join(directory, new_filename)
        # 重命名文件
        os.rename(old_filepath, new_filepath)
        print(f"已将 '{filename}' 重命名为 '{new_filename}'")


def rename_and_sort_txt_files(directory , start_index):
    """
    重新命名当前目录中的所有.jpg文件，使它们按照12345...的顺序进行命名，并进行排序。

    Args:
        directory (str): 当前目录路径。
    """
    # 获取当前目录中所有.jpg文件的文件名列表
    jpg_files = [filename for filename in os.listdir(directory) if filename.endswith('.txt')]

    # 按文件名中的数字进行排序
    jpg_files.sort()

    # 逐个重命名.jpg文件
    for i, filename in enumerate(jpg_files, start_index):
        # 构建新文件名，使用当前索引作为数字部分
        new_filename = f"{i}.txt"
        # 构建旧文件的完整路径
        old_filepath = os.path.join(directory, filename)
        # 构建新文件的完整路径
        new_filepath = os.path.join(directory, new_filename)
        # 重命名文件
        os.rename(old_filepath, new_filepath)
        print(f"已将 '{filename}' 重命名为 '{new_filename}'")

def jpg_rename_files(directory_path, start_number=1):
    # 获取当前目录下所有的.jpg文件
    jpg_files = [f for f in os.listdir(directory_path) if f.endswith('.jpg')]
    # 根据文件名排序
    jpg_files.sort()

    # 对每个文件进行重新命名
    for index, file_name in enumerate(jpg_files, start=start_number):
        new_file_name = f"{index}.jpg"
        os.rename(os.path.join(directory_path, file_name), os.path.join(directory_path, new_file_name))
        print(f"Renamed: {file_name} to {new_file_name}")


def txt_rename_files(directory_path, start_number=1):
    # 获取当前目录下所有的.jpg文件
    jpg_files = [f for f in os.listdir(directory_path) if f.endswith('.txt')]
    # 根据文件名排序
    jpg_files.sort()

    # 对每个文件进行重新命名
    for index, file_name in enumerate(jpg_files, start=start_number):
        new_file_name = f"{index}.txt"
        os.rename(os.path.join(directory_path, file_name), os.path.join(directory_path, new_file_name))
        print(f"Renamed: {file_name} to {new_file_name}")


# 要搜索的目录路径
image_path = "/home/wys/图片/Image_Py/4.3/images"
txt_path = "/home/wys/图片/Image_Py/4.3/labels"
start = 1

# 获取所有.jpg结尾的 文件名列表 并按文件名中的数字排序
#用2个列表存起来
#含后缀 .jpg .txt
jpg_files_sorted = get_jpg_filenames_sorted(image_path)
txt_files_sorted = get_txt_filenames_sorted(txt_path)

jpg_file_names_without_extension = []
txt_file_names_without_extension = []
different_numbers = []
file_names_with_extension = []

for file_name in jpg_files_sorted:
    # 使用split()函数将文件名按照'.'分割成文件名和后缀名的列表
    name_without_extension = file_name.split('.')[0]
    jpg_file_names_without_extension.append(name_without_extension)

for file_name in txt_files_sorted:
    # 使用split()函数将文件名按照'.'分割成文件名和后缀名的列表
    name_without_extension = file_name.split('.')[0]
    txt_file_names_without_extension.append(name_without_extension)

# # 找到两个 文件名列表  中不同的文件名，存入列表，并排序
difference_set = set(jpg_file_names_without_extension) ^ set(txt_file_names_without_extension) 
different_numbers = list(difference_set)

for file_name in different_numbers:
    # 添加 .txt 后缀
    file_name_with_extension = file_name + ".txt"
    file_names_with_extension.append(file_name_with_extension)


for file_name in file_names_with_extension:
    print(file_name)
   
# # 执行删除操作
delete_files(txt_path, file_names_with_extension)

# # 重新命名当前目录中的所有.jpg文件，使它们按照12345...的顺序进行命名，并进行排序
# rename_and_sort_jpg_files(directory_path_)

jpg_rename_files(image_path, start_number=start)
txt_rename_files(txt_path, start_number=start)


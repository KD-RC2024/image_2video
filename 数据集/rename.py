import os

def rename_files(folder_path, start_number):

    file_list = os.listdir(folder_path)
    
    # 遍历文件列表
    for index, filename in enumerate(file_list):
        if filename.endswith('.jpg'):
            # 生成新的文件名
            new_filename = '{}.jpg'.format(start_number + index)
            
            # 旧的文件路径
            old_path = os.path.join(folder_path, filename)
            
            # 新的文件路径
            new_path = os.path.join(folder_path, new_filename)
            
            # 重命名文件
            os.rename(old_path, new_path)
            print("Renamed '{}' to '{}'".format(filename, new_filename))

# 指定文件夹路径和起始数字
folder_path = '/home/wys/桌面/image1'
start_number =  8000 # 开始数字

# 调用函数重命名文件
rename_files(folder_path, start_number)


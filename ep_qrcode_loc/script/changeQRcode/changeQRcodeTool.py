import tkinter as tk
from tkinter import messagebox

docpath = "/home/zl/work/ep-qrcode-loc/src/ep_qrcode_loc/script/changeQRcode/your_document.txt"

class TextSearchReplaceApp:
    def __init__(self, root):
        self.root = root
        self.root.title("中力-二维码调试工具")

        # 创建输入框和按钮
        self.label_a = tk.Label(root, text="输入编号A:")
        self.label_a.grid(row=0, column=0, padx=10, pady=10)
        self.entry_a = tk.Entry(root, width=50)
        self.entry_a.grid(row=0, column=1, padx=10, pady=10)
        self.button_a = tk.Button(root, text="查找编号A", command=self.search_string_a)
        self.button_a.grid(row=0, column=2, padx=10, pady=10)
        self.button_ym = tk.Button(root, text="角度-0.1", command=self.yaw_minus_string_a)
        self.button_ym.grid(row=0, column=3, padx=10, pady=10)
        self.button_yp = tk.Button(root, text="角度+0.1", command=self.yaw_plus_string_a)
        self.button_yp.grid(row=0, column=4, padx=10, pady=10)

        self.label_b = tk.Label(root, text="输入编号B:")
        self.label_b.grid(row=1, column=0, padx=10, pady=10)
        self.entry_b = tk.Entry(root, width=50)
        self.entry_b.grid(row=1, column=1, padx=10, pady=10)
        self.button_b = tk.Button(root, text="替换编号A为B", command=self.replace_string_a_with_b)
        self.button_b.grid(row=1, column=2, padx=10, pady=10)

        # 创建文本框用于显示结果
        self.result_text = tk.Text(root, height=20, width=80)
        self.result_text.grid(row=2, column=0, columnspan=3, padx=10, pady=10)

        # 初始化文档内容
        self.document_lines = []
        self.document_lines_temp = []
        self.load_document()

    def load_document(self):
        try:
            with open(docpath, "r", encoding="utf-8") as file:
                self.document_lines = file.readlines()
        except FileNotFoundError:
            messagebox.showerror("错误", "文档" + docpath +"未找到！")
            self.document_lines = []

    def search_string_a(self):
        search_string = self.entry_a.get()
        if not search_string:
            messagebox.showwarning("警告", "请输入编号A！")
            return

        self.result_text.delete(1.0, tk.END)  # 清空结果框
        found = False
        
        if not search_string.isdigit():
            messagebox.showwarning("警告", "请输入数字！")
            return

        for i, line in enumerate(self.document_lines):
            if search_string in line:
                found = True
                start_line = max(0, i - 5)
                end_line = min(len(self.document_lines), i + 6)
                context_lines = self.document_lines[start_line:end_line]
                self.result_text.insert(tk.END, f"找到字符串A所在的行（行号：{i + 1}）及其上下文：\n")
                self.result_text.insert(tk.END, "".join(context_lines))
                self.result_text.insert(tk.END, "\n\n")

        if not found:
            messagebox.showinfo("未找到", f"文档中未找到字符串 '{search_string}'")
            
    def yaw_minus_string_a(self):
        search_string = self.entry_a.get()
        if not search_string:
            messagebox.showwarning("警告", "请输入编号A！")
            return

        self.result_text.delete(1.0, tk.END)  # 清空结果框
        found = False
        
        if not search_string.isdigit():
            messagebox.showwarning("警告", "请输入数字！")
            return
        
        self.result_text.delete(1.0, tk.END)  # 清空结果框
        found = False
       
        self.document_lines_temp = []
        
        # 遍历每一行，查找目标字符串
        for line in self.document_lines:
            if search_string in line:
                # 找到最后一个逗号的位置
                last_comma_index = line.rfind(',')
                if last_comma_index != -1:
                    
                    yaw = float(line[(last_comma_index+1):-1]) - 0.1
                    
                    # 删除最后一个逗号后面的内容，并在末尾添加 "0"
                    line = line[:(last_comma_index+1)] + str(yaw) + "\n"
            #print(line)
            self.document_lines_temp.append(line)
        self.document_lines = self.document_lines_temp
        
        # 将修改后的内容写回文件
        with open(docpath, 'w', encoding='utf-8') as file:
            file.writelines(self.document_lines)
        

        # 显示替换后字符串B的上下文
        self.result_text.delete(1.0, tk.END)  # 清空结果框
        found = False

        for i, line in enumerate(self.document_lines):
            if search_string in line:
                found = True
                start_line = max(0, i - 5)
                end_line = min(len(self.document_lines), i + 6)
                context_lines = self.document_lines[start_line:end_line]
                self.result_text.insert(tk.END, f"替换后的编号B所在的行（行号：{i + 1}）及其上下文：\n")
                self.result_text.insert(tk.END, "".join(context_lines))
                self.result_text.insert(tk.END, "\n\n")

        if not found:
            messagebox.showinfo("未找到", f"文档中未找到编号 '{search_string}'")

        messagebox.showinfo("成功", "编号"+str(search_string)+"方向角减小0.1")

    def yaw_plus_string_a(self):
        search_string = self.entry_a.get()
        if not search_string:
            messagebox.showwarning("警告", "请输入编号A！")
            return

        self.result_text.delete(1.0, tk.END)  # 清空结果框
        found = False
        
        if not search_string.isdigit():
            messagebox.showwarning("警告", "请输入数字！")
            return
        
        self.result_text.delete(1.0, tk.END)  # 清空结果框
        found = False
       
        self.document_lines_temp = []
        
        # 遍历每一行，查找目标字符串
        for line in self.document_lines:
            if search_string in line:
                # 找到最后一个逗号的位置
                last_comma_index = line.rfind(',')
                if last_comma_index != -1:
                    
                    yaw = float(line[(last_comma_index+1):-1]) + 0.1
                    
                    # 删除最后一个逗号后面的内容，并在末尾添加 "0"
                    line = line[:(last_comma_index+1)] + str(yaw) + "\n"
            #print(line)
            self.document_lines_temp.append(line)
        self.document_lines = self.document_lines_temp
        
        # 将修改后的内容写回文件
        with open(docpath, 'w', encoding='utf-8') as file:
            file.writelines(self.document_lines)
        

        # 显示替换后字符串B的上下文
        self.result_text.delete(1.0, tk.END)  # 清空结果框
        found = False

        for i, line in enumerate(self.document_lines):
            if search_string in line:
                found = True
                start_line = max(0, i - 5)
                end_line = min(len(self.document_lines), i + 6)
                context_lines = self.document_lines[start_line:end_line]
                self.result_text.insert(tk.END, f"替换后的编号B所在的行（行号：{i + 1}）及其上下文：\n")
                self.result_text.insert(tk.END, "".join(context_lines))
                self.result_text.insert(tk.END, "\n\n")

        if not found:
            messagebox.showinfo("未找到", f"文档中未找到编号 '{search_string}'")

        messagebox.showinfo("成功", "编号"+str(search_string)+"方向角增大0.1")

    def replace_string_a_with_b(self):
        search_string = self.entry_a.get()
        replace_string = self.entry_b.get()
        if not search_string or not replace_string:
            messagebox.showwarning("警告", "请输入字符串A和字符串B！")
            return
        
        if not search_string.isdigit() or not replace_string.isdigit():
            messagebox.showwarning("警告", "编号A和编号B请输入数字！")
            return
        
        # 替换字符串A为字符串B
        self.document_lines = [line.replace(search_string, replace_string) for line in self.document_lines]

        # # 保存修改后的文档
        # with open(docpath, "w", encoding="utf-8") as file:
        #     file.writelines(self.document_lines)
            

        # with open(docpath, 'r', encoding='utf-8') as file:
        #     lines = file.readlines()
        
        self.document_lines_temp = []
        
        # 遍历每一行，查找目标字符串
        for line in self.document_lines:
            if replace_string in line:
                # 找到最后一个逗号的位置
                last_comma_index = line.rfind(',')
                if last_comma_index != -1:
                    # 删除最后一个逗号后面的内容，并在末尾添加 "0"
                    line = line[:(last_comma_index+1)] + "0\n"
            #print(line)
            self.document_lines_temp.append(line)
        self.document_lines = self.document_lines_temp
        
        # 将修改后的内容写回文件
        with open(docpath, 'w', encoding='utf-8') as file:
            file.writelines(self.document_lines)
        

        # 显示替换后字符串B的上下文
        self.result_text.delete(1.0, tk.END)  # 清空结果框
        found = False

        for i, line in enumerate(self.document_lines):
            if replace_string in line:
                found = True
                start_line = max(0, i - 5)
                end_line = min(len(self.document_lines), i + 6)
                context_lines = self.document_lines[start_line:end_line]
                self.result_text.insert(tk.END, f"替换后的字符串B所在的行（行号：{i + 1}）及其上下文：\n")
                self.result_text.insert(tk.END, "".join(context_lines))
                self.result_text.insert(tk.END, "\n\n")

        if not found:
            messagebox.showinfo("未找到", f"文档中未找到字符串 '{replace_string}'")

        messagebox.showinfo("成功", "替换完成！文档已更新。")


if __name__ == "__main__":
    root = tk.Tk()
    app = TextSearchReplaceApp(root)
    root.mainloop()
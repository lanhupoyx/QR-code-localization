time stamp    # 时间辍
string sender # 相机IP
string hex    # 数据帧内容（16进制）
string head   # 帧头
string sum    # 字节校验和
uint32 index     # 序号
float64 duration    # 算法耗时

uint32 code    # 二维码编号
float64 error_x   # 中心坐标x
float64 error_y   # 中心坐标y
float64 error_yaw # 中心yaw

bool is_decode       # 是否译码成功
bool is_accurate_loc # 是否精准定位成功
bool is_broad_loc    # 是否粗定位成功
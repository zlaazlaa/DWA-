#include<cstdio>
#include<vector>
#include<cmath>
#include<algorithm>

#define Max_Range 100 //栅格图是6000*6000
#define Radius 2.0 //小车半径2米不能有障碍物
#define pi 3.1416
#define One_Block_meter 0.05 //一格代表0.05米
/////
#define Safe_Distance 2//安全距离为2格（10厘米）
/////
#define DeltaT 1 //模拟时间长度
#define alpha -0.01 //夹角的参数
#define beta -0.1 //与A*路径距离的参数
#define gamma 3 //速度的参数
using namespace std;

typedef struct Pair {
    double Velocity;
    double Angular_Acceleration;
} Pair;

bool Legal_Coordinate(int x) {
    if (x > Max_Range || x < 0) return false;
    return true;
}

bool
If_Ok(bool map[Max_Range][Max_Range], int Car_x, int Car_y, double Now_Angle, double Velocity, double Angular_Velocity,
      double Max_Acceleration) {
    //能否在障碍物前停下？能则返回True
    //    double Break_Time = Velocity / Max_Acceleration;//刹车时间
    //    double X_Velocity = Velocity * cos((pi / 180) * Now_Angle);//x轴方向速度
    ///////
//    double Break_Length = pow(Velocity, 2) / (2 * Max_Acceleration);//刹车距离
    //////
    double Break_Length = Velocity * DeltaT;
    double New_angle = (pi / 180) * (Now_Angle + Angular_Velocity * DeltaT);//Delta时间后的角度（弧度制）
    double SIN = sin(New_angle);
    double COS = cos(New_angle);
    double X_Length = Break_Length * COS;//x轴方向刹车距离
    int X_Blocks = (int) (X_Length / One_Block_meter);//x轴方向刹车的格数
    double Radius_Blocks = Radius / One_Block_meter;//小车的半径占多少格
    int Y_Radius_Blocks = abs((int) (Radius_Blocks / COS));//小车y轴方向占用多少格
    Y_Radius_Blocks /= 2;//上下各占一半
    Y_Radius_Blocks += Safe_Distance;//遍历范围增加安全距离
    for (int i = 1; i <= X_Blocks; i++) {
        int j = (int) (i * SIN);
        for (int w = -Y_Radius_Blocks; w <= Y_Radius_Blocks; w++) {
            if (!Legal_Coordinate(Car_x + i) || !Legal_Coordinate(Car_y + j + w)) { //坐标超出范围
                continue;
            }
            if (map[Car_x + i][Car_y + j + w]) {
                return false;
            }
        }
    }
    return true;
}

typedef struct Coordinate {//栅格图内坐标（非物理坐标）
    int x;
    int y;
} Coordinate;

double Distance(int a, int b, int x, int y) {
    //求栅格图中两点距离（非物理距离）
    return sqrt(pow(a - x, 2) + pow(b - y, 2));
}

int Get_H(Coordinate Left, Coordinate Right) {
    return abs(Left.x - Right.x) + abs(Left.y - Right.y);
}

vector<Coordinate> A_Star_Road;

double Get_A_Star_Dist(int New_Car_x, int New_Car_y, int Move_x, int Move_y) {
    //改成坐标距离吧，算起来太麻烦了
    //改成广度优先可以不，扩大半径
    //改成遍历部分vector?
    //得到DeltaT后距离A_Star路线的距离
    int Step_Number = 2 * (Move_x + Move_y) + 2;
    if (Step_Number > A_Star_Road.size()) {
        Step_Number = A_Star_Road.size();
    }
    double Dist = 2147483647;
    for (int i = 0; i < A_Star_Road.size(); i++) {
        double Temp = Get_H({New_Car_x, New_Car_y}, {A_Star_Road[i].x, A_Star_Road[i].y});//和求h的估计值通用，都是坐标距离
        if (Temp < Dist) {//取点到点最小距离作为点到线的距离
            Dist = Temp;
        }
    }
    return Dist;
}

bool Finish_Tag = false;//Open_List_Number定义在主函数外面时，和Finish_Tag互换定义的位置，会导致程序出错，头疼
//是因为Open_List数组开小了?究竟需要多少?空图是三倍，随机的一个是2.4倍
int Open_List_Number = 0;
double g_Score[Max_Range][Max_Range];
double h_Score[Max_Range][Max_Range];
double f_Score[Max_Range][Max_Range];
bool In_Open_List[Max_Range][Max_Range] = {false};
bool In_Close_List[Max_Range][Max_Range] = {false};
Coordinate Father[Max_Range][Max_Range];
Coordinate Open_List[3 * Max_Range];
int Dx[] = {1, 0, -1, 0, 1, -1, 1, -1};
int Dy[] = {0, 1, 0, -1, 1, -1, -1, 1};

bool cmp(Coordinate a, Coordinate b) {//降序，取Open_List[Open_List_Number - 1]为最小值
    return f_Score[a.x][a.y] > f_Score[b.x][b.y];
}

bool Legal_Coordinate(Coordinate x, bool Map[Max_Range][Max_Range]) {//判断坐标是否合法
    if (x.x > Max_Range || x.x < 0 || x.y > Max_Range || x.y < 0 || Map[x.x][x.y]) {
        return false;
    }
    return true;
}

void SORT() {
    sort(Open_List, Open_List + Open_List_Number, cmp);
}

bool A_Star_Judge_Legal(bool Map[Max_Range][Max_Range], int Now_x, int Now_y) {//A_Star加上这个
    for (int i = Now_x - Safe_Distance; i <= Now_x + Safe_Distance; i++) {
        for (int j = Now_y - Safe_Distance; j <= Now_y + Safe_Distance; j++) {
            if (Now_x > Max_Range || Now_x < 0 || Now_y > Max_Range || Now_y < 0) continue;
            if (Map[i][j]) {
                return false;
            }
        }
    }
    return true;
}

void A_Star(bool Map[Max_Range][Max_Range], Coordinate Start, Coordinate End) {
    Open_List[Open_List_Number++] = Start;
    g_Score[Start.x][Start.y] = 0;
    h_Score[Start.x][Start.y] = Get_H(Start, End);
    f_Score[Start.x][Start.y] = g_Score[Start.x][Start.y] + h_Score[Start.x][Start.y];
    In_Open_List[Start.x][Start.y] = true;
    while (Open_List_Number) {
        if (In_Open_List[End.x][End.y]) {
            Finish_Tag = true;
            break;
        }
        Coordinate Now_Node = Open_List[Open_List_Number - 1];
        Open_List_Number--;//删除最小的点
        In_Open_List[Now_Node.x][Now_Node.y] = false;
        In_Close_List[Now_Node.x][Now_Node.y] = true;
        for (int i = 0; i < 7; i++) {
            double Add = 1;
            if (i > 3) Add = 1.4;//走斜线
            int Next_x = Now_Node.x + Dx[i];
            int Next_y = Now_Node.y + Dy[i];
            if (!A_Star_Judge_Legal(Map, Next_x, Next_y)) continue;//安全距离内有障碍物
            if (!Legal_Coordinate({Next_x, Next_y}, Map)) continue;//不合法
            if (In_Close_List[Next_x][Next_y]) continue;
            if (!In_Open_List[Next_x][Next_y]) {
                Open_List[Open_List_Number++] = {Next_x, Next_y};
                In_Open_List[Next_x][Next_y] = true;
                Father[Next_x][Next_y] = Now_Node;
                g_Score[Next_x][Next_y] = g_Score[Now_Node.x][Now_Node.y] + Add;
                h_Score[Next_x][Next_y] = Get_H({Next_x, Next_y}, End);
                f_Score[Next_x][Next_y] = g_Score[Next_x][Next_y] + h_Score[Next_x][Next_y];
                SORT();
            } else {
                if (g_Score[Next_x][Next_y] > g_Score[Now_Node.x][Now_Node.y] + Add) {
                    g_Score[Next_x][Next_y] = g_Score[Now_Node.x][Now_Node.y] + Add;
                    Father[Next_x][Next_y] = Now_Node;
                    f_Score[Next_x][Next_y] = g_Score[Next_x][Next_y] + h_Score[Next_x][Next_y];
                    SORT();
                }
            }
        }
    }
    if (!Finish_Tag) {
        printf("Error,Can not reach to the destination!\n");
        return;
    }
    Coordinate Temp = End;
    A_Star_Road.push_back(Temp);
    while (Temp.x != Start.x || Temp.y != Start.y) {
        Temp = Father[Temp.x][Temp.y];
        A_Star_Road.push_back(Temp);
    }
}

Pair DWA(bool Map[Max_Range][Max_Range], int Car_x, int Car_y, int Destination_x, int Destination_y, double Now_Angle,
         double Now_Velocity, double Now_Angular_Velocity, double Max_Angular_Acceleration, double Max_Velocity,
         double Max_Acceleration) {
    //栅格矩阵 小车的长度 小车的宽度 小车横坐标 小车纵坐标 中间目的地横坐标 中间目的地纵坐标 当前相对于栅格图角度 当前线速度 当前角速度 最大角加速度 最大速度 最大加速度
    typedef struct Sample {
        double Velocity;
        double Angular_Velocity;
        double Head;//角度差
        double A_Star_Dist;//与A*路径的距离
    } Sample;
    vector<Sample> Sample_Vector;
    A_Star(Map, {Car_x, Car_y}, {Destination_x, Destination_y});//求出A_Star_Road
    for (double Velocity = Now_Velocity - Max_Acceleration * DeltaT;
         Velocity <= Now_Velocity + Max_Acceleration * DeltaT; Velocity += 0.2) {
        for (double Angular_Velocity = Now_Angular_Velocity - Max_Angular_Acceleration * DeltaT;
             Angular_Velocity <= Now_Angular_Velocity + Max_Angular_Acceleration * DeltaT; Angular_Velocity += 0.1) {
            if (Velocity < 0) {//不考虑倒车
                continue;
            }
            //遍历速度和角速度
            if (Velocity > Max_Velocity || fabs(Angular_Velocity) > Max_Angular_Acceleration) { //超过小车性能，放弃采样
                continue;
            }
            if (!If_Ok(Map, Car_x, Car_y, Now_Angle, Velocity, Angular_Velocity, Max_Acceleration)) { //会碰到障碍物，放弃采样
                continue;
            }
            Sample Now_Sample;
//                        double Move_Length = Velocity * DeltaT;//DeltaT内移动的距离（物理距离）
//                        double New_Angle = (pi / 180) * (Now_Angle + Angular_Velocity * DeltaT);//(弧度制)
//                        int Move_x = (int) ((Move_Length * cos(New_Angle)) / One_Block_meter);//x轴方向移动的方格数量
//                        int Move_y = (int) ((Move_Length * sin(New_Angle)) / One_Block_meter);//y轴方向移动的方格数量
//                        int New_Car_x = Car_x + Move_x;//小车DeltaT后的新坐标
//                        int New_Car_y = Car_y + Move_y;//小车DeltaT后的新坐标
//                        double Head = fab(Now_Angle + Angular_Velocity * DeltaT -
//                                      (atan2(Destination_y - New_Car_y, Destination_x - New_Car_x) * 180.0 / pi));//角度偏差（角度制）
            int Move_x;
            int Move_y;
            if (Angular_Velocity != 0) {
                double r = Velocity / Angular_Velocity;
                Move_x = (int) ((Velocity * DeltaT * cos(0.01745 * (Now_Angle + Angular_Velocity * DeltaT))) /
                                One_Block_meter);
                Move_y = (int) ((Velocity * DeltaT * sin(0.01745 * (Now_Angle + Angular_Velocity * DeltaT))) /
                                One_Block_meter);
            } else {
                Move_x = (int) ((Velocity * DeltaT * cos(Now_Angle * 0.01745)) / One_Block_meter);
                Move_y = (int) ((Velocity * DeltaT * sin(Now_Angle * 0.01745)) / One_Block_meter);
            }
            int New_Car_x = Car_x + Move_x;//小车DeltaT后的新坐标
            int New_Car_y = Car_y + Move_y;//小车DeltaT后的新坐标
            double Head = fabs(Now_Angle + Angular_Velocity * DeltaT -
                               (atan2(Destination_y - New_Car_y, Destination_x - New_Car_x) * 57.3));//角度偏差（角度制）
            Now_Sample.Velocity = Velocity;
            Now_Sample.Angular_Velocity = Angular_Velocity;
            Now_Sample.Head = Head;
            Now_Sample.A_Star_Dist = Get_A_Star_Dist(New_Car_x, New_Car_y, Move_x, Move_y);
            printf("Dist   %lf\n", Now_Sample.A_Star_Dist);
            Sample_Vector.push_back(Now_Sample);
        }
    }
    double Sum_Of_Head = 0.00000001;
    double Sum_Of_A_Star_Dist = 0.00000001;
    double Sum_Of_Velocity = 0.00000001;
    for (auto &i : Sample_Vector) {
        Sum_Of_Velocity += i.Velocity;
        Sum_Of_A_Star_Dist += i.A_Star_Dist;
        Sum_Of_Head += i.Head;
    }
    double Target_Velocity = 0;
    double Target_Angular_Velocity = 0;
    double Max_Score = -2147483648;
    for (auto &i : Sample_Vector) {//求出评价函数值最大的vector
        double Now_Score = alpha * i.Head / Sum_Of_Head + beta * i.A_Star_Dist / Sum_Of_A_Star_Dist +
                           gamma * i.Velocity / Sum_Of_Velocity;
        if (Now_Score > Max_Score) {//更新Target
            Max_Score = Now_Score;
            Target_Velocity = i.Velocity;
            Target_Angular_Velocity = i.Angular_Velocity;
        }
    }
    Pair Target_Statement;//打包返回参数
    Target_Statement.Velocity = Target_Velocity;
    Target_Statement.Angular_Acceleration = Target_Angular_Velocity;
    return Target_Statement;
}

bool demo[Max_Range][Max_Range];
int MAP[Max_Range][Max_Range];

int main() {
    for (int i = 0; i <= 75; i++) {
        MAP[35][i] = 4;
        demo[35][i] = true;
    }
    Pair Target;
    Target = DWA(demo, 0, 0, Max_Range - 1, Max_Range - 1, 90, 1.5, 0, 20, 1.5, 5);
    for (int i = 0; i < A_Star_Road.size(); i++) {
        MAP[A_Star_Road[i].x][A_Star_Road[i].y] = 1;
    }
    for (int j = Max_Range - 1; j > 0; j--) {
        for (int i = 0; i < Max_Range - 1; i++) {
            printf("%d", MAP[i][j]);
        }
        printf("\n");
    }
    printf("Velocity : %lf\nAngular_Acceleration : %lf\n", Target.Velocity, Target.Angular_Acceleration);
    return 0;
}
/*


typedef struct Map {
    vector<Coordinate> A_Star;//全局最短路经过的点的坐标
    bool map[Max_Range][Max_Range];//是否有障碍物
};
typedef struct Robot_Model {
    double Now_Velocity;//当前速度
    double Now_Angular_Velocity;//当前角速度
    Coordinate Now_coordinate;//当前坐标
    double Now_Angular;//当前角度（相对于原点）

    double Max_Velocity;//最大正向速度     //先不考虑double Min_Velocity = 0;//最大倒车速度
    double Max_Angular_Velocity;//最大顺时针角速度
    double Min_Angular_Velocity;//最大逆时针角速度
    double Max_Acceleration;//最大加速度
    double Max_Angular_Acceleration;//最大角加速度
    double Bot_Radius;//小车车身半径

    double Velocity_Gradient;//速度采样梯度
    //double DeltaT;//时间间隔
    double Sum_Of_Velocity_Gradient = (Max_Velocity / Velocity_Gradient) * (Max_Velocity / 2);//速度采样总和（归一化用）
    double Angular_Velocity_Gradient;//角速度采样梯度
    double Sum_Of_Angular_Velocity_Gradient;//角速度采样总和（归一化用）
} Robot;

double Head(Coordinate Now, Coordinate Target) {//起点到终点斜率的绝对值
    return (atan2(Target.y - Now.y, Target.x - Now.x) * 180.0 / 3.1416);
}
*/
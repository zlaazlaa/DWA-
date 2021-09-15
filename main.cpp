#include <iostream>
#include<vector>
#include<cmath>
#include<cstring>
#include<algorithm>

using namespace std;
namespace Get_DWA_Answer {
#define Max_Range 120
#define Delta 0.1
#define Predict_Delta 1.0
#define Velocity_Accuracy 0.2
#define Angular_Velocity_Accuracy 0.5
#define One_Block 0.05
#define Safe_Distance 2
#define Alpha 1 //Obstacle
#define Beta (-1.0) //Goal
#define Gamma 1.0 //Velocity
#define Delta2 (-5) //Dist_To_A_Star
    typedef struct Bot_Model {
        double Max_Velocity;
        double Max_Angular_Velocity;
        double Max_Velocity_Acceleration;
        double Max_Angular_Acceleration;
    } Bot_Model;
    typedef struct Pair {
        double Target_Velocity;
        double Target_Angular_Velocity;
    } Pair;
    typedef struct Coordinate {
        int x;
        int y;
    } Coordinate;
    typedef struct Node {
        double Dist_To_Obstacle;
        double Dist_To_Goal;
        double Velocity;
        double Angular_Velocity;
        double VELOCITY;
        double ANGULAR_VELOCITY;
        double Dist_To_A_Star;
    } Node;
    vector<Coordinate> Obstacle_Set;
    vector<Node> Ok_List;
    vector<Coordinate> Trajectory;

    double MIN(double a, double b) {
        return a > b ? b : a;
    }

    double MAX(double a, double b) {
        return a < b ? b : a;
    }

    double Calc_Dist(Coordinate a, Coordinate b) {
        return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
    }

    bool Legal_Coordinate(Coordinate x) {
        if (x.x < 0 || x.x >= Max_Range || x.y < 0 || x.y >= Max_Range) return false;
        else return true;
    }

    bool Get_Trajectory(Coordinate Car_Coordinate, double Now_Velocity, double Now_Angle, double Now_Angular_Velocity) {
        double Car_x = One_Block * Car_Coordinate.x;
        double Car_y = One_Block * Car_Coordinate.y;
        double Time_Sum = 0;
        while (Time_Sum <= Predict_Delta) {
            Time_Sum += Delta;
            double Next_Angle = Now_Angle + Now_Angular_Velocity * Delta;
            Car_x += Now_Velocity * cos(Next_Angle * 0.017453292) * Delta;
            Car_y += Now_Velocity * sin(Next_Angle * 0.017453292) * Delta;
            Now_Angle = Next_Angle;
            if (Legal_Coordinate({lround(Car_x / One_Block), lround(Car_y / One_Block)})) {
                Trajectory.push_back({lround(Car_x / One_Block), lround(Car_y / One_Block)});
            } else {
                return false;
            }
        }
        if (Trajectory.empty()) return false;
        return true;
    }

    double Get_Dist_To_Obstacle() {
        if (Obstacle_Set.empty()) {
            return -1;
        }
        double minn = 1e100;
        for (auto &i: Trajectory) {
            for (auto &j: Obstacle_Set) {
                double Dist = Calc_Dist(i, j);
                minn = MIN(minn, Dist);
            }
        }
        return minn;
    }

    double Get_Dist_To_Goal(Coordinate Car_Destination) {
        return Calc_Dist(Trajectory[Trajectory.size() - 1], Car_Destination);
    }

    bool Finish_Tag = false;
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
    vector<Coordinate> A_Star_Road;

    int Get_H(Coordinate Left, Coordinate Right) {
        return abs(Left.x - Right.x) + abs(Left.y - Right.y);
    }

    bool A_Star_Judge_Legal(bool Map[Max_Range][Max_Range], int Now_x, int Now_y) {//A_Star加上这个
        for (int i = Now_x - Safe_Distance; i <= Now_x + Safe_Distance; i++) {
            for (int j = Now_y - Safe_Distance; j <= Now_y + Safe_Distance; j++) {
                if (i >= Max_Range || i < 0 || j >= Max_Range || j < 0) continue;
                if (Map[i][j]) {
                    return false;
                }
            }
        }
        return true;
    }

    bool cmp(Coordinate a, Coordinate b) {//降序，取Open_List[Open_List_Number - 1]为最小值
        return f_Score[a.x][a.y] > f_Score[b.x][b.y];
    }

    void SORT() {
        if (Open_List_Number <= 1) return;
        sort(Open_List, Open_List + Open_List_Number, cmp);
    }

    double Get_A_Star_Dist(Coordinate End_Road) {
        //改成坐标距离吧，算起来太麻烦了
        //改成广度优先可以不，扩大半径
        //改成遍历部分vector?
        //得到DeltaT后距离A_Star路线的距离
//        int Step_Number = 2 * (Move_x + Move_y) + 2;
//        if (Step_Number > A_Star_Road.size()) {
//            Step_Number = A_Star_Road.size();
//        }
        double Dist = INT_MAX;
        for (auto &i: A_Star_Road) {
            double Temp = Get_H(End_Road, i);//和求h的估计值通用，都是坐标距离
            if (Temp < Dist) {//取点到点最小距离作为点到线的距离
                Dist = Temp;
            }
        }
        return Dist;
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
                if (!Legal_Coordinate({Next_x, Next_y})) continue;//不合法
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

    void Refresh_Programme() {
        Obstacle_Set.clear();
        Ok_List.clear();
        Finish_Tag = false;
        Open_List_Number = 0;
        memset(g_Score, 0, sizeof(g_Score));
        memset(h_Score, 0, sizeof(h_Score));
        memset(f_Score, 0, sizeof(f_Score));
        memset(In_Open_List, false, sizeof(In_Open_List));
        memset(In_Close_List, false, sizeof(In_Close_List));
        A_Star_Road.clear();
    }

    Pair DWA(char Map[Max_Range][Max_Range], Coordinate Car_Coordinate, double Now_Angle, double Now_Velocity,
             double Now_Angular_Velocity, Coordinate Car_Destination, Bot_Model Model) {
        Refresh_Programme();
        for (int i = 0; i < Max_Range; i++)
            for (int j = 0; j < Max_Range; j++)
                if (Map[i][j] != '0' && Map[i][j] != '7' && Map[i][j] != '9') {
                    /////////////////////
                    //这个不等于‘7’要改，测试的时候加上的。
                    Obstacle_Set.push_back({i, j});
                }
        bool MAP[Max_Range][Max_Range];
        for (int i = 0; i < Max_Range; i++) {
            for (int j = 0; j < Max_Range; j++) {
                if (Map[i][j] == '0' || Map[i][j] == '7') {
                    MAP[i][j] = false;
                } else {
                    MAP[i][j] = true;
                }
            }
        }
        A_Star(MAP, Car_Coordinate, Car_Destination);
        double MIN_Dist_To_Obstacle = 1e100;
        double MAX_Dist_To_Obstacle = -10;
        double MIN_Dist_To_Goal = 1e100;
        double MAX_Dist_To_Goal = -10;
        double MIN_Velocity = 1e100;
        double MAX_Velocity = -1e100;
        double MIN_Angular_Velocity = 1e100;
        double MAX_Angular_Velocity = -1e100;
        double MIN_Dist_To_A_Star = 1e100;
        double MAX_Dist_To_A_Star = -10;
        for (double Velocity = Now_Velocity - Predict_Delta * Model.Max_Velocity_Acceleration;
             Velocity <=
             Now_Velocity + Predict_Delta * Model.Max_Velocity_Acceleration; Velocity += Velocity_Accuracy) {
            for (double Angular_Velocity = Now_Angular_Velocity - Predict_Delta * Model.Max_Angular_Acceleration;
                 Angular_Velocity <= Now_Angular_Velocity + Predict_Delta *
                                                            Model.Max_Angular_Acceleration; Angular_Velocity += Angular_Velocity_Accuracy) {
                if (fabs(Velocity) > Model.Max_Velocity || fabs(Angular_Velocity) > Model.Max_Angular_Velocity ||
                    Velocity < 0)
                    continue;
                Trajectory.clear();
                if (!Get_Trajectory(Car_Coordinate, Velocity, Now_Angle, Angular_Velocity)) continue;
                double Break_Length = ((Velocity * Velocity / (2 * Model.Max_Velocity_Acceleration)) / One_Block);
                double Dist_To_Obstacle = Get_Dist_To_Obstacle();
                if (Dist_To_Obstacle != -1 && Dist_To_Obstacle < Safe_Distance) continue;
                if (Dist_To_Obstacle != -1 && Break_Length > Dist_To_Obstacle) continue;
                double Dist_To_Goal = Get_Dist_To_Goal(Car_Destination);
                double Dist_To_A_Star = Get_A_Star_Dist(Trajectory[Trajectory.size() - 1]);
                MIN_Dist_To_Obstacle = MIN(Dist_To_Obstacle, MIN_Dist_To_Obstacle);
                MAX_Dist_To_Obstacle = MAX(Dist_To_Obstacle, MAX_Dist_To_Obstacle);
                MIN_Dist_To_Goal = MIN(Dist_To_Goal, MIN_Dist_To_Goal);
                MAX_Dist_To_Goal = MAX(Dist_To_Goal, MAX_Dist_To_Goal);
                MIN_Velocity = MIN(Velocity, MIN_Velocity);
                MAX_Velocity = MAX(Velocity, MAX_Velocity);
                MIN_Angular_Velocity = MIN(Angular_Velocity, MIN_Angular_Velocity);
                MAX_Angular_Velocity = MAX(Angular_Velocity, MAX_Angular_Velocity);
                MIN_Dist_To_A_Star = MIN(Dist_To_A_Star, MIN_Dist_To_A_Star);
                MAX_Dist_To_A_Star = MAX(Dist_To_A_Star, MAX_Dist_To_A_Star);
                Ok_List.push_back(
                        {Dist_To_Obstacle, Dist_To_Goal, Velocity, Angular_Velocity, Velocity, Angular_Velocity,
                         Dist_To_A_Star});
            }
        }
        for (auto &i: Ok_List) {
            i.Dist_To_Obstacle =
                    (i.Dist_To_Obstacle - MIN_Dist_To_Obstacle) / (MAX_Dist_To_Obstacle - MIN_Dist_To_Obstacle + 1e-8);
            i.Dist_To_Goal = (i.Dist_To_Goal - MIN_Dist_To_Goal) / (MAX_Dist_To_Goal - MIN_Dist_To_Goal + 1e-8);
            i.Velocity = (i.Velocity - MIN_Velocity) / (MAX_Velocity - MIN_Velocity + 1e-8);
            i.Angular_Velocity =
                    (i.Angular_Velocity - MIN_Angular_Velocity) / (MAX_Angular_Velocity - MIN_Angular_Velocity + 1e-8);
            i.Dist_To_A_Star =
                    (i.Dist_To_A_Star - MIN_Dist_To_A_Star) / (MAX_Dist_To_A_Star - MIN_Dist_To_A_Star + 1e-8);
        }
        double MAX_Score = -1e100;
        Pair Target = {0, 0};
        for (auto &i: Ok_List) {
            double Now_Score = 0;
//            Now_Score += Alpha * i.Dist_To_Obstacle;
            Now_Score += Beta * i.Dist_To_Goal;
            Now_Score += Gamma * i.Velocity;
            Now_Score += Delta2 * i.Dist_To_A_Star;
            if (Now_Score > MAX_Score) {
                MAX_Score = Now_Score;
                Target.Target_Angular_Velocity = i.ANGULAR_VELOCITY;
                Target.Target_Velocity = i.VELOCITY;
            }
        }
        return Target;
    }
}
char MM[Max_Range][Max_Range] = {'0'};
using namespace Get_DWA_Answer;

int main() {
    freopen("out.txt", "w", stdout);
    for (int i = 0; i < Max_Range; i++) {
        for (int j = 0; j < Max_Range; j++) {
            MM[i][j] = '0';
        }
    }
//    for(int i = 8;i <= 8;i ++) {
//        for(int j = 20;j <= 100;j ++) {
//            MM[i][j] = '1';
//        }
//    }
    for (int i = 50; i <= 52; i++) {
        for (int j = 10; j <= 60; j++) {
            MM[i][j] = '1';
        }
    }
    for (int i = 40; i <= 50; i++) {
        for (int j = 10; j <= 12; j++) {
            MM[i][j] = '1';
        }
    }
    for (int i = 40; i <= 50; i++) {
        for (int j = 58; j <= 60; j++) {
            MM[i][j] = '1';
        }
    }
//    for (int i = 30; i <= Max_Range - 30; i++) {
//        for (int j = Max_Range - 13; j <= Max_Range - 12; j++) {
//            MM[i][j] = '1';
//        }
//    }
//    for (int j = Max_Range - 1; j >= 0; j--) {
//        for (int i = 0; i < Max_Range - 1; i++) {
//            printf("%c", MM[i][j]);
//        }
//        printf("\n");
//    }
    Get_DWA_Answer::Pair Target;
    Get_DWA_Answer::Coordinate Car_Coordinate = {0, 35};
    Get_DWA_Answer::Coordinate Car_Destination = {119, 35};
//    MM[Car_Destination.x][Car_Destination.y] = '9';
    double Angle = 50;
    double Velocity = 0;
    double Angular_Velocity = 0;
    Get_DWA_Answer::Bot_Model Model = {1.0, 60, 0.5, 60};
    for (int i = 1; i <= 20; i++) {
        Target = Get_DWA_Answer::DWA(MM, Car_Coordinate, Angle, Velocity, Angular_Velocity, Car_Destination, Model);
        double Car_x = One_Block * Car_Coordinate.x;
        double Car_y = One_Block * Car_Coordinate.y;
        double Time_Sum = 0;
        while (Time_Sum <= Predict_Delta) {
            Time_Sum += Delta;
            double Next_Angle = Angle + Target.Target_Angular_Velocity * Delta;
            Car_x += Target.Target_Velocity * cos(Next_Angle * 0.017453292) * Delta;
            Car_y += Target.Target_Velocity * sin(Next_Angle * 0.017453292) * Delta;
            MM[lround(Car_x / One_Block)][lround(Car_y / One_Block)] = '7';
            Angle = Next_Angle;
        }
        MM[lround(Car_x / One_Block)][lround(Car_y / One_Block)] = '7';
        Velocity = Target.Target_Velocity;
        Angular_Velocity = Target.Target_Angular_Velocity;
        Car_Coordinate.x = lround(Car_x / One_Block);
        Car_Coordinate.y = lround(Car_y / One_Block);
//        for (int j = Max_Range - 1; j >= 0; j--) {
//            for (int i = 0; i < Max_Range; i++) {
//                printf("%c", MM[i][j]);
//            }
//            printf("\n");
//        }
//        printf("-------------------------------------------------------------------------\n");
//        printf("-------------------------------------------------------------------------\n");
    }
    for (int j = Max_Range - 1; j >= 0; j--) {
        for (int i = 0; i < Max_Range; i++) {
            if (MM[i][j] != '0') printf("%c", MM[i][j]);
            else {
                printf(" ");
            }
        }
        printf("\n");
    }
//    printf("%lf  %lf\n", Target.Target_Velocity, Target.Target_Angular_Velocity);
    std::cout << "Hello, World!" << std::endl;
    return 0;
}
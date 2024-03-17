#include "avoidance.h"

Avoidance::
    Avoidance()
{
    double o_lat = 47.3977423;
    double o_lon = 8.5455930;
    double o_alt = 535.4;

    initialiseReference(o_lat, o_lon, o_alt);

    for (int i = 0; i < N_REG; i++)
    {
        neighbor[i].id = 0;
        neighbor[i].pri = true;
        neighbor[i].avoiding = false;
        neighbor[i].collision = false;
        neighbor[i].terminate = false;
        neighbor[i].pri_checked = false;
        neighbor[i].a_state = 0;
        neighbor[i].chk_set[0] = 0;
        neighbor[i].chk_set[1] = 0;
        neighbor[i].chk_set[2] = 0;
        neighbor[i].v_r_norm = -1.0;
    }
    neighbor_count = 0;
    myinfo.hx = 0.0f;
    myinfo.hy = 0.0f;
    myinfo.hfx = 0.0f;
    myinfo.hfy = 0.0f;

    v0 = 10.0;
    bank_angle = M_PI / 6.0;
    d_m = 20.0;
    rate = 20.0;
    dt = 1 / rate;
    rho_min = pow(v0, 2) / tan(bank_angle) / 9.81;
    d_m_more = d_m + 4.0 * rho_min; //Threat Checker에서, 현재 거리가 얘보다 작아야지만 위협등록함.

    trigger = false;
    chk_wp_set = false;
}

Avoidance::
    ~Avoidance()
{
}

// node별로 v0,d_m,rho_min,rate등의 물리적 스펙이 다를 것이기때문에, node에서 이것을 받아온다.
void Avoidance::
    initializeSpecification(double v0_i, double bank_angle_i, double d_m_i, double rate_i)
{
    double v0_big; // v0가 10이하일 경우, rho_min은 그냥 속도 10인 경우로 가정하고 계산하기위함.
    if (v0_i > v0)
        v0_big = v0_i;
    else
        v0_big = v0;
    v0 = v0_i;
    bank_angle = bank_angle_i;
    d_m = d_m_i;
    rate = rate_i;
    dt = 1 / rate;
    rho_min = pow(v0_big, 2) / tan(bank_angle) / 9.81;
    d_m_more = d_m + 4.0 * rho_min; //Threat Checker에서, 현지 위치가 이 거리 안이어야지만 위협등록함.
}

//neighbor들을 검사해서 위협갯수가 1개이상인지 체크.
bool Avoidance::
    threat_checker()
{
    int threat_count = 0;
    bool decending = false;
    int k = 0;
    double dt = 0.1;
    if (neighbor_count == 0)
    {
        printf("[Threat Checker] Neighbor:0\n");
        return false;
    }
    else
    {
        for (int i = 0; i < neighbor_count; i++)
        {
            if (neighbor[i].avoiding)
            {
                printf("[Threat Checker] Neighbor[%d].avoiding.\n", i);
                threat_count++;
            }
            else
            {
                // Check it is threat or not
                float current_dist = sqrt(pow((neighbor[i].lx - myinfo.lx), 2) + pow((neighbor[i].ly - myinfo.ly), 2));
                float fixed_dist = current_dist;
                float next_dist = sqrt(pow(((neighbor[i].lx + dt * neighbor[i].vx) - (myinfo.lx + dt * myinfo.vx)), 2) + pow(((neighbor[i].ly + dt * neighbor[i].vy) - (myinfo.ly + dt * myinfo.vy)), 2));
                float next_dist_min = next_dist;
                // Check one step, and if the distance is too close, it becomes threat
                if (next_dist < TH_R)
                {
                    neighbor[i].cx = myinfo.lx + myinfo.vx * dt;
                    neighbor[i].cy = myinfo.ly + myinfo.vy * dt;
                    neighbor[i].avoiding = true;
                    threat_count++;
                    printf("[Threat checker] Threat registered. but too close.\n");
                    continue;
                }
                // Even though they are not close, if the distance decrease, it still possible to be threat.
                if (current_dist > next_dist)
                {
                    decending = true;
                    k = 1;
                    printf("[Threat Chekcer] neighbor[%d] is approaching.\n", i);
                }
                else
                {
                    decending = false;
                    printf("[Threat Checker] neighbor[%d] is receding.\n", i);
                }
                // Check future traj. until they are close enough, or they go apart.
                while (decending)
                {
                    k = k + 1;
                    current_dist = next_dist;
                    next_dist = sqrt(pow(((neighbor[i].lx + (k * dt * neighbor[i].vx)) - (myinfo.lx + (k * dt * myinfo.vx))), 2) + pow(((neighbor[i].ly + (k * dt * neighbor[i].vy)) - (myinfo.ly + (k * dt * myinfo.vy))), 2));
                    if (next_dist < next_dist_min)
                        next_dist_min = next_dist;

                    if (next_dist < TH_R)
                    {
                        neighbor[i].cx = myinfo.lx + (k * dt * myinfo.vx);
                        neighbor[i].cy = myinfo.ly + (k * dt * myinfo.vy);
                        if (fixed_dist < d_m_more)
                        {
                            neighbor[i].avoiding = true;
                            threat_count++;
                            printf("[Threat checker] Threat registered. (%.2f, %.2f), time remained = %f, vel = %.2f, %.2f\n", neighbor[i].cx, neighbor[i].cy, k * dt, myinfo.vx, myinfo.vy);
                            break;
                        }
                    }
                    if (current_dist > next_dist)
                    {
                        decending = true;
                    }
                    else
                    {
                        decending = false;
                        //printf("[Threat checker] Approaching, but will not crush.");
                    }
                }
                printf("[Threat Checker] nextdist_min :%.2f\n", next_dist_min);
            }
        }
    }
    if (threat_count == 0)
    {
        printf("[Threat checker] Threat:0\n");
        return false;
    }
    else
    {

        printf("[Threat checker] Threat:%d\n", threat_count);
        return true;
    }
}

//avoiding 상태인 위협의 priority 체크
bool Avoidance::
    priority_check()
{
    int avoid_count = 0;
    for (int i = 0; i < neighbor_count; i++)
    {
        if (neighbor[i].avoiding && !neighbor[i].pri_checked)
        {
            if (myinfo.type == 1)
            { // If I am rotary wing.
                float lr_test = -atan2(myinfo.vy, myinfo.vx) + atan2(neighbor[i].vy, neighbor[i].vx);
                lr_test = ThetaRange(lr_test);
                printf("[priority_check] (positive means avoid.) %.2f, r_u[%d] (%.2f, %.2f)/ v_u0 (%.2f, %.2f)\n", lr_test, i, (neighbor[i].lx - myinfo.lx), (neighbor[i].ly - myinfo.ly), myinfo.vx, myinfo.vy);

                if (lr_test < 0) // neighbor is my left. I have a priority.
                    neighbor[i].pri = true;
                else // neighbor is my right.
                    neighbor[i].pri = false;
            }
            else if (myinfo.type == 2)
            { // If I am fixed wing.
                if (neighbor[i].type == 1)
                    neighbor[i].pri = true;
                else if (neighbor[i].type == 2)
                {
                    if (((neighbor[i].lx - myinfo.lx) * myinfo.vy - (neighbor[i].ly - myinfo.ly) * myinfo.vx) < 0) // neighbor is my left. I have a priority.
                        neighbor[i].pri = true;
                    else // neighbor is my right.
                        neighbor[i].pri = false;
                }
            }
            neighbor[i].pri_checked = true;
        }
        if (neighbor[i].pri == false)
            avoid_count++;
    }

    if (avoid_count == 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

// priority가 없는(1) 충돌위험(2) neighbor를 순서대로 배열로 저장하고, 회피기동함수인 vec_calc를 불러옴.
Vector_t
Avoidance::
    avoid()
{
    // find crash point and number
    int num_crash = 0;
    int neighbor_list[5] = {
        0,
    };
    for (int i = 0; i < neighbor_count; i++)
    {
        if (1) //neighbor[i].avoiding)
        {
            if (1) //!neighbor[i].pri)
            {
                //temp_cp[0][num_crash] = neighbor[i].cx;
                //temp_cp[1][num_crash] = neighbor[i].cy;
                neighbor_list[num_crash] = i;
                num_crash = num_crash + 1;
            }
        }
    }

    Vector_t vel_out;

    vel_out = vec_calc(neighbor_list, num_crash);

    return vel_out;
}

void Avoidance::
    neighbor_managing()
{
    // Remove terminated neighbors
    for (int i = 0; i < N_REG; i++)
    {
        if (neighbor[i].terminate)
        {
            for (int j = i + 1; j < N_REG; j++)
            {
                neighbor[j - 1] = neighbor[j];
            }
            neighbor[N_REG - 1].id = 0;
            neighbor[N_REG - 1].pri = true;
            neighbor[N_REG - 1].avoiding = false;
            neighbor[N_REG - 1].terminate = false;
            i = i - 1;
        }
    }
    int count = 0;
    // Count neighbors
    for (int i = 0; i < N_REG; i++)
    {
        if (neighbor[i].id != 0)
        {
            count = count + 1;
        }
    }
    neighbor_count = count;
}

void
    Avoidance::
        //neighbor_update(int id, float stamp, double lat, double lon, double height, float vx, float vy, float vz) {
    neighbor_update(int id, int type, float stamp, double lat, double lon, double alt, float vx, float vy, float vz)
{
    // array check with id
    bool inlist = false;
    bool empty_slot = false;
    double t_x, t_y, t_z;
    geodetic2Ned(lat, lon, alt, &t_x, &t_y, &t_z);
    int i = 0;
    int j = 0;
    for (i = 0; i < N_REG; i++)
    {
        if (neighbor[i].id == id)
        {
            inlist = true;
            break;
        }
    }
    if (inlist)
    {
        neighbor[i].stamp = stamp;

        neighbor[i].lx = t_y;
        neighbor[i].ly = t_x;
        neighbor[i].lz = -t_z;
        neighbor[i].vx = vx;
        neighbor[i].vy = vy;
        neighbor[i].vz = vz;
        double r0, r_r[2];
        r_r[0] = t_y - myinfo.lx;
        r_r[1] = t_x - myinfo.ly;
        r0 = VectorNorm(r_r, 2);
        printf("[neighbor_update] r[%d]:  %.2f, %.2f (r0:%.2f), v: %.2f, %.2f  neighbor_count : %d\n", i, t_y, t_x, r0, vx, vy, neighbor_count);
    }
    else
    {
        for (j = 0; j < N_REG; j++)
        {
            if (neighbor[j].id == 0)
            {
                empty_slot = true;
                break;
            }
        }
        if (empty_slot)
        {
            neighbor[j].id = id;
            neighbor[j].stamp = stamp;
            neighbor[j].type = type;

            neighbor[i].lx = t_y;
            neighbor[i].ly = t_x;
            neighbor[i].lz = -t_z;
            neighbor[j].vx = vx;
            neighbor[j].vy = vy;
            neighbor[j].vz = vz;
            neighbor_count = j + 1;
            printf("[Neighbor Update] New neighbor updated in %d slot.\n", j);
        }
        else
        {
            printf("[Neighbor Update] No empty slot for neighbor. New detected neighbor cannot be registered.\n");
        }
    }
}

void Avoidance::
    myinfo_update(float stamp, int type, double lat, double lon, double height, float vx, float vy, float vz)
{

    double t_x, t_y, t_z;
    myinfo.stamp = stamp;
    myinfo.type = type;
    myinfo.lat = lat;
    myinfo.lon = lon;
    myinfo.height = height;
    geodetic2Ned(lat, lon, height, &t_x, &t_y, &t_z);
    myinfo.lx = t_y;
    myinfo.ly = t_x;
    myinfo.lz = -t_z;
    myinfo.vx = vx;
    myinfo.vy = vy;
    myinfo.vz = vz;
    printf("[myinfo_update] time: %f,   r_u0:%.2f,%.2f,    v_u0: %.2f %.2f\n", stamp, myinfo.lx, myinfo.ly, vx, vy);
}

void Avoidance::
    myinfo_wp_update(mavros_msgs::Waypoint waypoint_f, mavros_msgs::Waypoint waypoint) // INPUT : 내 WP LIST, 현재 몇번째(=i번째)를 돌고있는지.
{
    double t_x, t_y, t_z, t_x_f, t_y_f, t_z_f; // ned transfored. f = from(이전WP)
    double lat, lon, alt, lat_f, lon_f, alt_f; // llh INPUT

    lat = waypoint.x_lat;
    lon = waypoint.y_long;
    alt = waypoint.z_alt;
    lat_f = waypoint_f.x_lat;
    lon_f = waypoint_f.y_long;
    alt_f = waypoint_f.z_alt;

    geodetic2Ned(lat, lon, alt, &t_x, &t_y, &t_z);
    geodetic2Ned(lat_f, lon_f, alt_f, &t_x_f, &t_y_f, &t_z_f);

    //float check = sqrt(pow((t_y - myinfo.hx), 2) + pow((t_x - myinfo.hy), 2));
    //hfx 업데이트
    myinfo.hfx = t_y_f;
    myinfo.hfy = t_x_f;

    //hx 업데이트
    myinfo.hx = t_y;
    myinfo.hy = t_x;
    printf("[wp_update] hf : %.2f, %.2f,  h: %.2f, %.2f\n", myinfo.hfx, myinfo.hfy, myinfo.hx, myinfo.hy);
}

// [vec_calc] 회피알고리즘. u_v0 계산. //
Vector_t
Avoidance::
    vec_calc(int neighbor_list[], int num_crash)
{
    //INPUT
    double r_u0[2], r_u1[2], v_u0[2], v_u0_2[num_crash][2], v_u1[2], r_collision[2], r_waypoint[2], r_waypoint_prev[2]; // ( + .chk_set,d_m, rho_min,dt,v0 ...etc )

    //In-function Variables
    double r_r[2], r_rwp[2], v_r[2];
    double r0, th0, t_critical;
    double th1[101], th_range, theta_threshold, theta_sp, theta_sp0, cost[101], cost_min; //코스트함수 생성을위한 변수
    bool check, check_prev;                                                               //첫진입시기 체크를 위한 변수
    bool V_UPDATE;                                                                        //회피여부

    double theta_incident, theta_return, rho_min_rel; //복귀 기동에 필요한 변수
    double r_u0_hf[2], r_h_hf[2];                     //위치벡터.

    double v_u0_old[2];
    double v0_old;
    double v0_norm;

    //OUTPUT is v_u0 (+.chkset, ...)

    r_u0[0] = myinfo.lx;
    r_u0[1] = myinfo.ly;
    v_u0[0] = myinfo.vx;
    v_u0[1] = myinfo.vy;

    r_waypoint[0] = myinfo.hx;
    r_waypoint[1] = myinfo.hy;
    r_waypoint_prev[0] = myinfo.hfx;
    r_waypoint_prev[1] = myinfo.hfy;

    v_u0_old[0] = v_u0[0];
    v_u0_old[1] = v_u0[1];

    t_critical = 1000.0;
    double th_sp_init[num_crash] = {
        0.0,
    };

    VectorMinus(r_u0, r_waypoint_prev, r_u0_hf);
    VectorMinus(r_waypoint, r_waypoint_prev, r_h_hf);

    for (int i = 0; i < num_crash; i++)
    {
        v_u0_2[i][0] = v_u0[0];
        v_u0_2[i][1] = v_u0[1];
    }

    th_range = -M_PI / 2;

    theta_threshold = 1000.0;
    //theta_threshold = sqrt(pow(v_u0[0], 2) + pow(v_u0[1], 2)) / rho_min * dt * 40.0; // omega_max * dt  *40.0 (VERY LARGE)
    theta_incident = GetTheta(r_u0_hf) - GetTheta(r_h_hf);
    theta_incident = ThetaRange(theta_incident);
    theta_return = GetTheta(v_u0) - GetTheta(r_h_hf);
    theta_return = ThetaRange(theta_return);

    // v_u0 스케일링
    //VectorScaling(v_u0,v0);
    V_UPDATE = 0;

    for (int i = 0; i < num_crash; i++) //[AV1] th_sp_init[i]들 계산. (이후 th_sp얻어줌)
    {
        double r_r[2], v_r[2], v_r_old[2], r_hf_col[2], theta_hf_col, th_colhf_wpline, v_r_norm;

        r_u1[0] = neighbor[neighbor_list[i]].lx;
        r_u1[1] = neighbor[neighbor_list[i]].ly;
        r_collision[0] = neighbor[neighbor_list[i]].cx;
        r_collision[1] = neighbor[neighbor_list[i]].cy;
        v_u1[0] = neighbor[neighbor_list[i]].vx;
        v_u1[1] = neighbor[neighbor_list[i]].vy;

        check = neighbor[neighbor_list[i]].chk_set[0];
        check_prev = neighbor[neighbor_list[i]].chk_set[1];

        VectorMinus(r_u1, r_u0, r_r);
        VectorMinus(v_u0, v_u1, v_r);
        VectorMinus(v_u0_old, v_u1, v_r_old);
        VectorMinus(r_collision, r_waypoint_prev, r_hf_col);

        double v_r_copy[2], v_u0_copy[2];
        double theta_increment;
        double v_r_copy_norm;
        int k, idx_v_r;
        double v_r_max = 0.0;
        for (k = 0; k < 101; k++)
        {
            v_u0_copy[0] = v_u0[0];
            v_u0_copy[1] = v_u0[1];

            theta_increment = -M_PI / 180 * 75 * (k / 100.0) + M_PI / 6;
            VectorRotateTheta(v_u0_copy, theta_increment);
            VectorMinus(v_u0_copy, v_u1, v_r_copy);
            v_r_copy_norm = VectorNorm(v_r_copy, 2);
            if (v_r_max < v_r_copy_norm)
            {
                idx_v_r = k;
                v_r_max = v_r_copy_norm;
            }
        }

        rho_min_rel = pow(v_r_max, 2) / 9.81 / tan(bank_angle);
        //AV2(복귀)를 위한 변수들
        theta_hf_col = GetTheta(r_hf_col);                             // 이전 wp에서 충돌지점까지 그은 벡터의 각도
        th_colhf_wpline = ThetaRange(theta_hf_col - GetTheta(r_h_hf)); // WP line과, 이전wp에서 충돌지점을 그은 line 사이 각도

        check_prev = check;
        check = 0;

        if (CheckCollision(r_r, v_r, d_m, dt))
            neighbor[neighbor_list[i]].collision = true;
        else
            neighbor[neighbor_list[i]].collision = false;

        printf("\n[vec_calc] neighbor_list[%d]=%d. r_r : (%.2f, %.2f), r0:%3.2f, v_r : (%.2f,%.2f),  check: %d check_prev: %d, rho_min_rel: %.2f, v_r_max: %.2f (idx_v_r:%d)\n", i, neighbor_list[i], r_r[0], r_r[1], VectorNorm(r_r, 2), v_r[0], v_r[1], check, check_prev, rho_min_rel, v_r_max, idx_v_r);
        if (chk_is_it_right_side(th_colhf_wpline, r_hf_col) && !neighbor[neighbor_list[i]].pri) // wp선 좌측으로 d_m넘어가면 충돌무시하는 함수. priority가 없는경우에만회피.
        {
            if (VectorNorm(r_r, 2) < d_m + 2.2 * rho_min_rel)
                check = 1;
            if (neighbor[neighbor_list[i]].collision || VectorNorm(r_r, 2) < d_m) // r_r<d_m일때도 충돌알고리즘을 돌림.
            {
                r0 = VectorNorm(r_r, 2);
                th0 = GetTheta(v_r) - GetTheta(r_r);
                th0 = ThetaRange(th0);
                t_critical = (r0 - 2.0 * rho_min_rel - d_m) / VectorNorm(v_r, 2);
                if (t_critical <= 0)
                {
                    th_sp_init[i] = vec_calc_AV1(r_r, v_r, v_u0, v_u1, rho_min_rel, th0, theta_threshold, t_critical, i, v_u0_2); //INPUT r_r,v_r,v_u1,rho_min_rel,th0,t_critical // infuction th1[101],cost[101],cost_min,idx, th_r,th_sp_init_o, v_r_2,v_u0_2
                    V_UPDATE = 1;                                                                                                 //OUTPUT th_sp_init,v_u0_2;
                }
            }
        }
        neighbor[neighbor_list[i]].chk_set[0] = check;
        neighbor[neighbor_list[i]].chk_set[1] = check_prev;
    }

    int col_count = 0;
    for (int l = 0; l < num_crash; l++)
    {
        if (neighbor[neighbor_list[l]].collision)
        {
            col_count++;
            printf("[AV] col_count++\n");
        }
    }

    printf("[AV]  col_count : %d\n", col_count);
    if (V_UPDATE) // [AV1 마무리] AV1 한 번이라도 기동시 시행. th_sp_init 절댓가ㅄ이 가장 큰 것을 th_sp로 설정
    {
        theta_sp = th_sp_init[0];
        int idx = 0;
        for (int i = 0; i < num_crash; i++)
        {
            if (abs(theta_sp) < abs(th_sp_init[i]))
            {
                theta_sp = th_sp_init[i];
                idx = i;
            }
        }

        v_u0[0] = v_u0_2[idx][0];
        v_u0[1] = v_u0_2[idx][1];
    }
    else if (col_count == 0) //[AV2] : V변경 불필요시, wpline으로 복귀. 1단계회피이후, ( WP재설정점이후 + V변경 불필요).
    {
        theta_sp = vec_calc_AV2(v_u0, r_u0_hf, r_h_hf, theta_incident, theta_return, neighbor_list, num_crash);
        if (theta_sp > M_PI_2)
            theta_sp = M_PI_2;
        if (theta_sp < -M_PI_2)
            theta_sp = -M_PI_2;

        VectorRotateTheta(v_u0, SaturationFunc(theta_sp, theta_threshold));
    }

    VectorScaling(v_u0, v0); // AV1가 돌때는 필요없지만, 회피 거리이내로 진입했을때나, AV2가 돌고있을때는 의미가있음.

    printf("\n[AV%d]  INPUT:v0[0]=%4.1f v0[1]=%4.1f, OUTPUT: v0[0]=%4.1f v0[1]=%4.1f (theta_sp:%.4f).\n", V_UPDATE, v_u0_old[0], v_u0_old[1], v_u0[0], v_u0[1], theta_sp);
    printf("                        WP_set:%d, r_waypoint:(%.2f,%.2f),r_waypoint_prev:(%.2f,%.2f), ncrash : %d, nlist[0] : %d", chk_wp_set, r_waypoint[0], r_waypoint[1], r_waypoint_prev[0], r_waypoint_prev[1], num_crash, neighbor_list[0]);
    if (num_crash >= 2)
        printf("nlist[1] : %d", neighbor_list[1]);
    printf("\n");

    Vector_t vel;
    vel.x = v_u0[0];
    vel.y = v_u0[1];
    vel.z = 0.0f;
    return vel;
}

// AV1 회피기동 : 상대좌표계 정보들을 받아서, th_sp_abs(각도변화 셋포인트)를 return
double Avoidance::vec_calc_AV1(double r_r[], double v_r[], double v_u0[], double v_u1[], double rho_min_rel, double th0, double theta_threshold, double t_critical, int i, double v_u0_2[][2])
{
    //INPUT r_r,v_r,rho_min_rel,th0,t_critical // infuction-var : th1[101],cost[101],cost_min,idx, th_r,th_sp_init_o, v_r_2,v_u0_2
    //OUTPUT th_sp_init,V_UPDATE,chk_AV1break

    // #1. cost 함수계산해서 최소인 각 th_sp_rel 얻음.
    double th1[101], cost[101], cost_min, r0, th_r_rev, v_r_2[2], th_range;
    int idx_min_cost;
    double th_sp_rel, th_sp_abs;

    th_range = -M_PI / 2.0;
    r0 = VectorNorm(r_r, 2);

    for (int j = 0; j < 101; j++)
    {
        th1[j] = th0 + j * th_range / 100.0;
        cost[j] = 1000.0;
        if (d_m < r0 * abs(sin(th1[j])))
            cost[j] = abs(th1[j] - th0);
    }
    cost_min = 10000.0;
    for (int j = 0; j < 101; j++)
    {
        if (cost[j] < cost_min)
        {
            cost_min = cost[j];
            idx_min_cost = j;
        }
    }
    th_sp_rel = th1[idx_min_cost] - th0; // th_SP is setted.
    v_r_2[0] = v_r[0];
    v_r_2[1] = v_r[1];

    if (cost_min == 1000.0) //d_m 이내로 접근하였을 때, 무작정 우회전하는 것이 아니라, v_r을 r_r 반대방향으로 움직이게끔 th_SP_abs 를 넣는다. 만약 th_sp_abs가 90도를 초과할 경우 90도로 바꿔준다..
    {
        double r_r_rev[2];
        r_r_rev[0] = -r_r[0];
        r_r_rev[1] = -r_r[1];
        th_r_rev = GetTheta(r_r_rev);
        th_sp_rel = th_r_rev - atan2(v_r[1], v_r[0]);
    }

    //#2. 상대좌표계상의 th_sp_rel 이용하여, 절대좌표계상의 th_sp_abs 얻음. (v_r을 th_sp_rel만큼 돌리고, 그 크기를 find_v_r_norm함수 이용해서 계산. v_u0=v_r+v_u1이용하여 return할 v_u0(th_sp_abs) 얻는다.)
    double v_r_norm;
    th_sp_rel = ThetaRange(th_sp_rel);

    bool TH_CHANGED = 0;

    v_r_norm = FindNormOfV_r(v_u1, v_r_2, th_sp_rel); //(1)v_r_norm얻기.

    if (v_r_norm == -1.0) // 에러상황고려 #1. v_a > v_u이면서, th_sp가 커서 원과 접점이 발생하지 않을때, 현실적으로 가능한 최대한 먼 쪽의 th_sp_rel을 다시 얻어줌.
    {
        double v_u1_rev[2], th_sp_rel_old;
        th_sp_rel_old = th_sp_rel;
        v_u1_rev[0] = -v_u1[0];
        v_u1_rev[1] = -v_u1[1];

        double v_r_2_norm = VectorNorm(v_r_2, 2);
        double v_u1_norm = VectorNorm(v_u1, 2);
        v_r_norm = sqrt(pow(v_u1_norm, 2) - pow(v0, 2));
        th_sp_rel = -atan2(v0, v_r_norm) + (GetTheta(v_u1_rev) - GetTheta(v_r_2));

        printf("[AV1] th_sp_rel changed : %.2f - > %.2f\n", th_sp_rel_old, th_sp_rel);
        TH_CHANGED = 1;
    }
    double th_sp_rel_max = acos((pow(v_r_norm, 2) + pow(VectorNorm(v_r, 2), 2) - (pow(VectorNorm(v_u0, 2), 2) + pow(v0, 2))) / 2.0 / VectorNorm(v_r, 2) / v_r_norm);
    printf(" [AV1] th_sp_rel_max : %.2f, th_sp_rel :%.2f\n", th_sp_rel_max, th_sp_rel);

    if (abs(th_sp_rel) > th_sp_rel_max) // 에러상황고려 #2. th_sp_rel이 너무 크다보니 th_abs가 90도를 넘어버릴때, 90도가 되게끔, th_sp_rel을 다시 설정해줌. 이걸이용하여 다시 v_r_norm구하면됨. ( 주로 v_a>v_u일때 발생)
    {
        TH_CHANGED = 1;
        double th_sp_rel_old2 = th_sp_rel;
        th_sp_rel = th_sp_rel_max * th_sp_rel / abs(th_sp_rel);

        printf("[AV1] th_sp_rel is over th_sp_rel_max : %.2f -> %.2f\n", th_sp_rel_old2, th_sp_rel);
    }
    if (TH_CHANGED) // 위 두 에러상황발생시, 새로얻은 th_sp_rel을 이용해서 v_r의 크기를 다시계산해줌
        v_r_norm = FindNormOfV_r(v_u1, v_r_2, th_sp_rel);

    VectorRotateTheta(v_r_2, th_sp_rel); // (2) v_r을 th_sp_rel만큼 돌림.
    VectorScaling(v_r_2, v_r_norm);      // (3) 계산해서얻은 v_r의 크기를 적용.

    VectorPlus(v_u1, v_r_2, v_u0_2[i]);                                                  // (4) v_u0=v_u1 + v_r 을 이용해서 v_u0을 얻음으로써 완료.
    th_sp_abs = ThetaRange(atan2(v_u0_2[i][1], v_u0_2[i][0]) - atan2(v_u0[1], v_u0[0])); // 몇도 돌렸는지(th_sp_abs) 역산.

    v_u0_2[i][0] = v_u0[0];
    v_u0_2[i][1] = v_u0[1];
    VectorRotateTheta(v_u0_2[i], th_sp_abs);

    printf("[AV1]\n");
    printf("cost[%d]=%4.2f (=%4.2f), th_1[]-th0=%4.2f,th_sp_init_rel:%.3f,th_sp_init_abs[%d]=%.3f,th0=%4.3f tc=%4.2f, v_u0_2[%d]=(%.2f,%.2f)\n", idx_min_cost, cost[idx_min_cost], cost_min, th1[idx_min_cost] - th0, th_sp_rel, i, th_sp_abs, th0, t_critical, i, v_u0_2[i][0], v_u0_2[i][1]);

    return th_sp_abs;
}

// [AV2] WP line상으로 복귀. rho외부에선 WP라인에 수직하게 접근하고, rho내부에선 둘사이 거리를 INPUT으로 수직방향속도를 OUTPUT으로 하는 P control.
double Avoidance::vec_calc_AV2(double v_u0[], double r_u0_hf[], double r_h_hf[], double theta_incident, double theta_return, int neighbor_list[], int num_crash)
{
    double theta_sp; //OUTPUT
    double v0_2 = VectorNorm(v_u0, 2);
    double rho = pow(v0_2, 2) / 9.81 / tan(bank_angle);

    //double r=sqrt(2)/(sqrt(2)-1.0)*rho;
    double d_h = -sin(theta_incident) * VectorNorm(r_u0_hf, 2) / sin(theta_return); //속도벡터로 내위치에서 WP line까지이은 거리. Negative(-) value는 UAV가 WP line에서 멀어지고있다는 의미.]
    double d_hv = sin(abs(theta_incident)) * VectorNorm(r_u0_hf, 2);                // 경로점 직선까지의 수직거리(수직성분).
    double d_hh = sqrt(pow(rho, 2) - pow(rho - d_hv, 2));                           // rho를 반지름으로하는 원을 그렸을때 수평좌표

    double theta_sp0 = atan2(r_h_hf[1], r_h_hf[0]) - atan2(v_u0[1], v_u0[0]); //WP LINE을 따라가도록 하는 각도와의 차이
    theta_sp = theta_sp0;

    if (d_hv > rho)             // (1) distance >rho_min 이면, 수직하게접근
        if (theta_incident < 0) //right case
            theta_sp += M_PI_2;
        else //left case
            theta_sp -= M_PI_2;
    else // (2) rho내부로 들어가면, 둘 사이 수직거리를 INPUT으로하는 P control.
    {
        double k = 1.0;
        double th_circle = acos(1.0 - d_hv / rho);

        if (theta_incident < 0.0)
            theta_sp += th_circle;
        else
            theta_sp -= th_circle;
    }

    if (d_hv < rho / 2.0 && abs(theta_sp0) < M_PI / 18.0) // distance <rho, angle<10 degree 이면 avoiding 해제. (피할 것이 없고, WP를 따라서 돎으로, 해제하는 논리.)
    {
        theta_sp = 0.0;
        chk_wp_set = 0;
        SetAvoidingFalse(neighbor_list, num_crash); //set all avoiding false
    }
    theta_sp = ThetaRange(theta_sp);
    printf("\n[AV2] rho:%.2f, d_hv:%.2f, d_h:%.2f, theta_incident:%.2f, theta_return:%.2f\n", rho_min, d_hv, d_h, theta_incident, theta_return);
    return theta_sp;
}

// 모든 neighbor의 avoiding을 해제하여, MISSION모드(+ state : 1)로 복귀하도록함. //
void Avoidance::SetAvoidingFalse(int neighbor_list[], int num_crash)
{
    for (int i = 0; i < num_crash; i++) //모든 avoiding 해제해서 MISSION복귀
    {
        printf("\n\n### Set ALL AVOIDING FALSE ### \n\n");
        neighbor[neighbor_list[i]].avoiding = false;
        neighbor[neighbor_list[i]].chk_set[0] = 0;
        neighbor[neighbor_list[i]].chk_set[1] = 0;
        neighbor[neighbor_list[i]].chk_set[2] = 0;
        neighbor[neighbor_list[i]].v_r_norm = -1.0;
        neighbor[neighbor_list[i]].pri_checked = false;
    }
}

// wp선 왼쪽으로 d_m까지의 충돌만 고려하고, 그 너머의 충돌은 무시한다. //
bool Avoidance::chk_is_it_right_side(double th_colhf_wpline, double r_hf_col[2])
{
    bool check = !(th_colhf_wpline > 0 && VectorNorm(r_hf_col, 2) * sin(th_colhf_wpline) > d_m);
    return check;
}

// 충돌체크 :: 최소거리 + 거리감소(접근/이탈)여부 검사
bool Avoidance::
    CheckCollision(double r[], double v[], double d, double dt)
{
    double th0 = atan2(v[1], v[0]) - atan2(r[1], r[0]);

    double r1[2];
    for (int i = 0; i < 2; i++)
    {
        r1[i] = r[i] - v[i] * dt;
    }
    double r0n = VectorNorm(r, 2);
    double r1n = VectorNorm(r1, 2);

    double min_dist = r0n * abs(sin(th0));
    if (min_dist <= d && r1n <= r0n)
    {
        printf("[Chk_Collision] : true. r0n*abs(sin)=%.2f,r_r=(%.2f,%.2f),v_r=(%.2f,%.2f) \n", min_dist, r[0], r[1], v[0], v[1]);
        return 1;
    }
    else
    {
        printf("[Chk_Collision] : false. r0n*abs(sin)=%.2f, r_r=(%.2f,%.2f),v_r=(%.2f,%2.f) \n", min_dist, r[0], r[1], v[0], v[1]);
        return 0;
    }
}

// 상대좌표계에서 세타SP에 따른, v_r의 크기를 결정하여 return한다. v_r벡터를 이 크기로 스케일링하여 v_u1에 더하면 publish할 v_u가 얻어짐.
double Avoidance::
    FindNormOfV_r(double v_u1[2], double v_r_2[2], double th_sp_rel)
{
    double x, a, b, c, th, x1, x2;
    double vec_tmp[2], vec_res[2];
    double vec_r2[2];

    double v_r[2];
    v_r[0] = v_r_2[0];
    v_r[1] = v_r_2[1];

    VectorRotateTheta(v_r, th_sp_rel);

    th = GetTheta(v_r);

    a = 1.0;
    b = 2.0 * (v_u1[0] * cos(th) + v_u1[1] * sin(th));
    c = pow(VectorNorm(v_u1, 2), 2) - pow(v0, 2);

    if (pow(b, 2) - 4.0 * a * c > 0) //접점이 존재.
    {
        x1 = (-b - sqrt(pow(b, 2) - 4.0 * a * c)) / 2.0 / a;
        x2 = (-b + sqrt(pow(b, 2) - 4.0 * a * c)) / 2.0 / a;
        // 둘중 양수이면서, 현재의 v_r크기와 가까운 가ㅄ을 선택(X) -> v_r크기와 sqrt(c)의 대수비교를통해 둘중하나를 선택.
        if (x1 > 0)
            if (VectorNorm(v_r, 2) < sqrt(c))
                x = x1;
            else
                x = x2;
        else if (x2 > 0)
            x = x2;
    }
    else //접점존재하지 않는 상황 (원이 작아서)
        x = -1.0;

    printf("[FindNormOfV_r] v_rn:%.2f\n", x);
    return x;
}

////////////  T  O  O  L  S  ////////////

/*******************************************
 * FUNCTION NAME : PURPOSE
 *   InverseMatrix() 함수 : 가우스-조던 소거법을 이용하여 역행렬 구하는 함수
 *
 * DESCRIPTION OF PARAMETERS
 *   double *work : 역행렬로 바꿀 행렬을 가리키는 포인터
 *   double *result : 역행렬로 변환된 값이 저장될 행렬을 가리키는 포인터
 *   int n : 행/열의 개수
 *
 * REMARKS
 *   double 형 데이터를 가지는
 *   2차원 배열에서만 사용할 수 있음
 *
 * SUBROUTINES AND FUNCTION SUBPROGRAMS REQUIRED
 *   CopyArray2D() 함수 : 2차원 배열을 복사하는 함수
 *******************************************/
int Avoidance::
    InverseMatrix(float *work, float *result, int n)
{
    int i, j, k;
    float constant;

    // 포인터 work 를 직접 조작하면,
    // 실제 원본 데이터값이 바뀌기 때문에 복사본을 만들어 작업한다.
    // 복사본으로 작업하기 위한 임시 매트릭스 선언
    float tmpWork[n][n];

    // 2차원 배열 복사
    CopyArray2D(work, &tmpWork[0][0], (n * n));

    // 계산 결과가 저장되는 result 행렬을 단위행렬로 초기화
    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++)
            result[i * n + j] = (i == j) ? 1 : 0;

    /* 대각 요소를 0 이 아닌 수로 만듦 */
    for (i = 0; i < n; i++)
        if (-ERROR < tmpWork[i][i] && tmpWork[i][i] < ERROR)
        {
            for (k = 0; k < n; k++)
            {
                if (-ERROR < tmpWork[k][i] && tmpWork[k][i] < ERROR)
                    continue;
                for (j = 0; j < n; j++)
                {
                    tmpWork[i][j] += tmpWork[k][j];
                    result[i * n + j] += result[k * n + j];
                }
                break;
            }
            if (-ERROR < tmpWork[i][i] && tmpWork[i][i] < ERROR)
                return 0;
        }

    /* Gauss-Jordan elimination */
    for (i = 0; i < n; i++)
    {
        // 대각 요소를 1로 만듦
        constant = tmpWork[i][i]; // 대각 요소의 값 저장
        for (j = 0; j < n; j++)
        {
            tmpWork[i][j] /= constant;     // tmpWork[i][i] 를 1 로 만드는 작업
            result[i * n + j] /= constant; // i 행 전체를 tmpWork[i][i] 로 나눔
        }

        // i 행을 제외한 k 행에서 tmpWork[k][i] 를 0 으로 만드는 단계
        for (k = 0; k < n; k++)
        {
            if (k == i)
                continue; // 자기 자신의 행은 건너뜀
            if (tmpWork[k][i] == 0)
                continue; // 이미 0 이 되어 있으면 건너뜀

            // tmpWork[k][i] 행을 0 으로 만듦
            constant = tmpWork[k][i];
            for (j = 0; j < n; j++)
            {
                tmpWork[k][j] = tmpWork[k][j] - tmpWork[i][j] * constant;
                result[k * n + j] = result[k * n + j] - result[i * n + j] * constant;
            }
        }
    }

    return (0);
}
/*******************************************
 * FUNCTION NAME : PURPOSE
 *      TransArray2d() 함수 : 2차원 배열을 Transpose하는 함수
 * 
 * DESCRIPTION OF PARAMETERS
 *   double *source : 소스 행렬
 *   double *target : 결과 행렬
 *   int row : 소스 행렬 행 개수 ( = 결과행렬 열 개수)
 *   int col : 소스 행렬 열 개수 ( = 결과행렬 행 개수)
 * 
 * REMARKS
 *   결과행렬의 크기를 검증하는 루틴없음 
 *   
 * SUBROUTINES AND FUNCTION SUBPROGRAMS REQUIRED
 *    NONE
*******************************************/
int Avoidance::TransArray2D(double *source, double *target, int row, int col)
{
    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < col; j++)
        {
            target[row * j + i] = source[col * i + j];
        }
    }
    return 0;
}
/*******************************************
 * FUNCTION NAME : PURPOSE
 *   CopyArray2D() 함수 : 2차원 배열을 복사하는 함수
 *
 * DESCRIPTION OF PARAMETERS
 *   double *source : 소스 행렬 (복사 원본)
 *   double *target : 결과 행렬 (복사 사본)
 *   int n : 데이터의 개수 (행 x 열)
 *
 * REMARKS
 *   double 형 데이터를 가지는
 *   2차원 배열에서만 사용할 수 있음
 *   복사되는 곳의 크기를 검증하는 루틴 없음.
 *
 * SUBROUTINES AND FUNCTION SUBPROGRAMS REQUIRED
 *   NONE
 *******************************************/
int Avoidance::
    CopyArray2D(float *source, float *target, int n)
{
    if (n < 1)
        return (-1);

    int i;
    for (i = 0; i < n; i++)
        target[i] = source[i];

    return (0);
}

/*******************************************
 * FUNCTION NAME : PURPOSE
 *   MulArray2D() 함수 : 2차원 배열을 곱하는 함수
 *
 * DESCRIPTION OF PARAMETERS
 *   double *work1 : 첫번째 작업 행렬
 *   double *work2 : 두번째 작업 행렬
 *   double *result : 행렬의 곱을 저장할 행렬
 *   int r1 : 첫번째 2차원 배열의 행의 개수
 *   int c1 : 첫번째 2차원 배열의 열의 개수 (두번째 2차원 배열의 행의 개수도 됨)
 *   int c2 : 두번째 2차원 배열의 열의 개수
 *
 * REMARKS
 *   NONE
 *
 * SUBROUTINES AND FUNCTION SUBPROGRAMS REQUIRED
 *   NONE
 *******************************************/
int Avoidance::
    MulArray2D(float *work1, float *work2, float *result, int r1, int c1, int c2)
{
    int i, j, k;

    for (i = 0; i < r1; i++)
    {
        for (j = 0; j < c2; j++)
        {
            result[i * c2 + j] = 0.0;
            for (k = 0; k < c1; k++)
            {
                result[i * c2 + j] += work1[i * c1 + k] * work2[k * c2 + j];
                //printf("w1[%2d*%2d+%2d:%2d](%f) * w2[%2d*%2d+%2d:%2d](%f) = %f \t result[%2d*%2d+%2d:%2d](%f)\n",
                //    i, c1, k, i*c1+k, work1[i*c1+k], k, c2, j, k*c2+j, work2[k*c2+j], (work1[i*c1+k]*work2[k*c2+j]), i, c2, j, i*c2+j, result[i*c2+j]);
            }
        }
    }

    return (0);
}

/*******************************************
 * FUNCTION NAME : PURPOSE
 *   InitMatrix() 함수 : 2차원 배열을 0 으로 초기화 하는 함수
 *
 * DESCRIPTION OF PARAMETERS
 *   double *work : 작업할 매트릭스의 주소값을 가지는 포인터 (2차원 배열을 1차원인 것처럼 처리)
 *   int row : 행의 개수
 *   int col : 열의 개수
 *
 * REMARKS
 *   데이터 형은 double 형이어야 함.
 *
 * SUBROUTINES AND FUNCTION SUBPROGRAMS REQUIRED
 *   NONE
 *******************************************/
int Avoidance::
    InitMatrix(float *work, int row, int col)
{
    int i, j;

    for (i = 0; i < row; i++)
        for (j = 0; j < col; j++)
            work[i * row + j] = 0.0;

    return (0);
}

/*******************************************
 * FUNCTION NAME : PURPOSE
 *   PrintMatrix() 함수 : 2차원 배열의 값들을 화면에 표시해 주는 함수
 *
 * DESCRIPTION OF PARAMETERS
 *   double *work : 작업할 매트릭스의 주소값을 가지는 포인터 (2차원 배열을 1차원인 것처럼 처리)
 *   int row : 행의 개수
 *   int col : 열의 개수
 *   char label : 설명 문자열
 *
 * REMARKS
 *   행렬의 크기는 상관 없으나, 데이터 형은 double 형이어야 함.
 *
 * SUBROUTINES AND FUNCTION SUBPROGRAMS REQUIRED
 *   NONE
 *******************************************/
int Avoidance::
    PrintMatrix(float *work, int row, int col, char *label)
{
    int i, j;

    printf("\n%s\n", label);
    for (i = 0; i < row; i++)
    {
        for (j = 0; j < col; j++)
        {
            printf("[%2d][%2d] : %.2f \t", i, j, work[i * col + j]);
        }
        printf("\n");
    }

    return (0);
}

Vector_t
Avoidance::
    vector_rotate(Vector_t vector, float angle)
{
    Vector_t rotate_vector;
    rotate_vector.x = vector.x * cosf(angle) - vector.y * sinf(angle);
    rotate_vector.y = vector.x * sinf(angle) + vector.y * cosf(angle);
    return rotate_vector;
}

float Avoidance::
    find_angle(Vector_t finding) //find angle_beta using euler's formula, [rad]
{
    float angle_beta;
    float temp_angle;
    if (finding.x > 0 && finding.y > 0)
    {
        temp_angle = finding.y / finding.x;
        angle_beta = atanf(temp_angle);
        //printf("1 %.2f\n", angle_beta);
    }
    else if (finding.x < 0 && finding.y > 0)
    {
        temp_angle = -finding.y / finding.x;
        angle_beta = M_PI - atanf(temp_angle);
        //printf("2 %.2f\n", angle_beta);
    }
    else if (finding.x < 0 && finding.y < 0)
    {
        temp_angle = finding.y / finding.x;
        angle_beta = atanf(temp_angle) + M_PI;
        //printf("3 %.2f\n", angle_beta);
    }
    else
    {
        temp_angle = -finding.y / finding.x;
        angle_beta = 2 * M_PI - atanf(temp_angle);
        //printf("4 %.2f\n", angle_beta);
    }

    return angle_beta;
}

Vector_t
Avoidance::
    normalization(Vector_t com_num)
{
    float std;
    Vector c;
    std = com_num.x * com_num.x + com_num.y * com_num.y;
    std = sqrt(std);
    c.x = com_num.x / std;
    c.y = com_num.y / std;
    return c;
}

Vector_t
Avoidance::
    divide_complex(Vector_t nomi_vector, Vector_t deno_vector)
{
    float denominator;
    Vector devide_vector;
    denominator = deno_vector.x * deno_vector.x + deno_vector.y * deno_vector.y;
    devide_vector.x = (nomi_vector.x * deno_vector.x + nomi_vector.y * deno_vector.y) / denominator;
    devide_vector.y = (nomi_vector.y * deno_vector.x - nomi_vector.x * deno_vector.y) / denominator;
    return devide_vector;
}

Vector_t
Avoidance::
    multiply_complex(Vector_t A, Vector_t B)
{
    Vector_t C;
    C.x = A.x * B.x - A.y * B.y;
    C.y = A.x * B.y + A.y * B.x;
    return C;
}

// Tool
// 벡터의 VectorNorm 얻음.
double
Avoidance::VectorNorm(double vec[], int n)
{
    double res = 0.0;
    for (int i = 0; i < n; i++)
        res += pow(vec[i], 2);
    res = sqrt(res);
    return res;
}

//saturation 함수 구현
double
Avoidance::SaturationFunc(double x, double x_sat)
{
    if (abs(x) < abs(x_sat))
        return x;
    else if (x < 0)
        return -x_sat;
    else if (x > 0)
        return x_sat;
}
//벡터를 세타만큼 돌림(라디안)
void Avoidance::VectorRotateTheta(double x[], double th)
{
    double x1[2];
    x1[0] = x[0];
    x1[1] = x[1];
    x[0] = cos(th) * x1[0] - sin(th) * x1[1];
    x[1] = sin(th) * x1[0] + cos(th) * x1[1];
    //return x2;
}
//각도의 범위를 -파이~파이 범위로 환산
double Avoidance::
    ThetaRange(double th)
{
    if (th > M_PI)
        th = th - 2 * M_PI;
    else if (th < -M_PI)
        th = th + 2 * M_PI;
    if (abs(th) > M_PI)
        th = ThetaRange(th);
    return th;
}
// 벡터 뺄셈연산. x-y를 z에 대입
void Avoidance::VectorMinus(double x[], double y[], double z[])
{
    z[0] = x[0] - y[0];
    z[1] = x[1] - y[1];
}
// 벡터 합연산. x+y를 z에 대입
void Avoidance::VectorPlus(double x[], double y[], double z[])
{
    z[0] = x[0] + y[0];
    z[1] = x[1] + y[1];
}
// 벡터의 norm을 v0로 스케일링.
void Avoidance::VectorScaling(double v[], double v0)
{
    double v_norm = VectorNorm(v, 2);
    v[0] *= v0 / v_norm;
    v[1] *= v0 / v_norm;
}
// 벡터의 각도를 얻음.
double Avoidance::GetTheta(double x[])
{
    double theta = atan2(x[1], x[0]);
    theta = ThetaRange(theta);
    return theta;
}
//위치좌표 출력하는 public함수
Vector_t
Avoidance::getdata_myinfo()
{
    Vector_t pose_data;

    pose_data.x = myinfo.lx;
    pose_data.y = myinfo.ly;
    return pose_data;
}

/****
 * FUNCTION NAME kalman_filter()
 * x : 위치+속도 행렬 4x1 (x,y방향순)
 * P : 공분산 행렬 4x4
 * 
 * A : 동역학모델 (상태전이 행렬) 4x4
 * H : 관측모델 4x4
 * Q : 동역학모델 오차 (공분산 행렬) 4x4 
 * R : 관측모델 오차 (공분산 행렬) 4x4
 * 
 * X_m,P_m : 예측단계가ㅄ 4x1
 * K : 칼만 이득 4x4
 * 
 *  
****/

int Avoidance::kalman_filter(double x[4], double P[4][4])
{ /*
    double x_m[4], P_m[4][4], K[4][4];
    double A[4][4], H[4][4],Q[4][4],R[4][4];
    double temp1[4][4],temp2[4][4];
    MulArray2D(&A[0][0],&x[0][0],x_m,4,4,1); // x_m=A*x;
    
    TransArray2D(&A[0][0],&temp1[0][0],4,4);
    MulArray2D(&P[0][0],&A[0][0],&temp2[0][0],4,4,4);
    MulArray2D



x_m=A*x;
P_m=A*P*A'+Q;

K=P_m*H'/(H*P_m*H'+R);

x=x_m+K*(z-H*x_m);
P=(eye(N)-K*H)*P_m;
*/
    return 0;
}
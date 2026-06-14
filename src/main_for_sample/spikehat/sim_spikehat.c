/**
 * sim_spikehat.c — MuJoCo シミュレーター上で spikehat API を模倣する実装
 *
 * spikehat.h と同じ API を提供し、実機の libspikehat と差し替えて使える。
 * 実機との違いはシリアル通信の代わりに MuJoCo の C API を使う点のみ。
 *
 * 【設計方針】
 *   - spikehat_t 構造体を拡張して MuJoCo の model/data を保持する
 *   - motor_get_position は累積角度（度数）を返す（実機エンコーダと同様）
 *   - time.sleep 相当は spikehat_motor_run_for_seconds 内でステップ実行
 *   - speed (-100〜+100) を ctrl (-1〜+1) に変換するスケール: 1/100
 *
 * 【ビルド方法】
 *   MuJoCo のヘッダーとライブラリをリンクする:
 *   cc -shared -fPIC -o libspikehat_sim.so sim_spikehat.c \
 *      -I/path/to/mujoco/include \
 *      -L/path/to/mujoco/lib -lmujoco \
 *      -lpthread -lm
 */

#include "robots/spikehat/spikehat_sim.h"

#include <mujoco/mujoco.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <time.h>

/* ── シミュレーション定数 ──────────────────────────────────────────── */

/** speed (-100〜+100) → ctrl (-1〜+1) の変換スケール */
#define SPEED_TO_CTRL  (1.0 / 100.0)

/** デフォルトの速度スケール（実時間1秒 = シム何秒分） */
#define DEFAULT_SPEED_SCALE  1.0

/** モータージョイント名 */
#define MOTOR_JOINT_NAME  "motor_joint"

/** 最大検出距離 [m] */
#define DIST_MAX_M  0.300

/** 距離測定不能値 [mm] */
#define DIST_INVALID  2000


/* ── spikehat_t の拡張フィールド ────────────────────────────────────
 *
 * spikehat.h の spikehat_t はシリアル通信用のフィールドを持つ。
 * シム版では fd / reader / lock / running / ports を流用しつつ、
 * MuJoCo 関連データを追加フィールドとして持つ。
 *
 * ただし spikehat_t の構造体を変更するとヘッダーが変わってしまうため、
 * 「シム専用の拡張構造体」を定義して spikehat_t * にキャストして使う。
 */

typedef struct sim_spikehat {
    /* spikehat_t と同じレイアウト（先頭に配置して互換性を保つ） */
    int                  fd;          /* 未使用（0固定） */
    pthread_t            reader;      /* 未使用 */
    pthread_mutex_t      lock;        /* 未使用 */
    spikehat_port_data_t ports[SPIKEHAT_MAX_PORTS];
    volatile int         running;

    /* MuJoCo 拡張フィールド */
    mjModel *model;
    mjData  *data;
    double   speed_scale;    /* 速度スケール（1.0=実時間と同期、>1で高速化） */
    double   ctrl;           /* 現在のctrl値 */
    double   next_wall;      /* 次のステップを実行すべき時刻[秒]（実時間ペーシング用、0=未初期化） */

    /* 累積角度管理（実機エンコーダ相当） */
    double   position_deg;   /* 累積角度 [度] */
    double   prev_qpos;      /* 前回のqpos値 [rad] */

    /* MuJoCo インデックスキャッシュ */
    int      joint_id;       /* motor_joint の ID */
    int      qpos_adr;       /* motor_joint の qpos アドレス */
    int      ctrl_id;        /* actuator の ctrl インデックス */
    int      color_site_id;    /* カラーセンサーsite（-1=未検出） */
    int      distance_site_id; /* 距離センサーsite（-1=未検出） */
    int      force_site_id;    /* フォースセンサーsite（-1=未検出） */
    int      force_sensor_id;  /* touchセンサーのID（-1=未検出） */
} sim_spikehat_t;


/* ── ヘルパー関数 ────────────────────────────────────────────────── */

static void _update_position(sim_spikehat_t *sim) {
    if (sim->qpos_adr < 0) return;  /* モーターなし */
    double curr = sim->data->qpos[sim->qpos_adr];
    double delta = curr - sim->prev_qpos;
    if (delta >  M_PI) delta -= 2.0 * M_PI;
    if (delta < -M_PI) delta += 2.0 * M_PI;
    /* gear の符号を取得して position_deg の増減方向を決める */
    double gear = (sim->ctrl_id >= 0)
                  ? sim->model->actuator_gear[sim->ctrl_id * 6]
                  : 1.0;
    double direction = (gear >= 0) ? 1.0 : -1.0;
    sim->position_deg += direction * delta * (180.0 / M_PI);
    sim->prev_qpos = curr;
}

/**
 * 1ステップ分の実時間が経過するまで待つ（speed_scaleで調整）。
 * 1ステップは timestep 秒のシミュレーション時間に相当するので、
 * speed_scale=1.0 なら実時間でも timestep 秒待つ（実機と同じ速度）。
 * speed_scale=10.0 なら timestep/10 秒待つ（10倍速）。
 * 処理が遅れている場合は基準時刻をリセットして暴走（早送り）を防ぐ。
 */
static void _pace(sim_spikehat_t *sim) {
    if (sim->speed_scale <= 0) return;

    double dt = sim->model->opt.timestep / sim->speed_scale;

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    double now_s = now.tv_sec + now.tv_nsec * 1e-9;

    if (sim->next_wall <= 0) {
        sim->next_wall = now_s;
    }
    sim->next_wall += dt;

    double remain = sim->next_wall - now_s;
    if (remain > 0) {
        struct timespec req;
        req.tv_sec  = (time_t)remain;
        req.tv_nsec = (long)((remain - (double)req.tv_sec) * 1e9);
        nanosleep(&req, NULL);
    } else {
        sim->next_wall = now_s;
    }
}

/** 1ステップ進めて累積角度を更新する */
static void sim_step(sim_spikehat_t *sim) {
    pthread_mutex_lock(&sim->lock);
    /* モーターのctrlのみ上書きする（他のアクチュエーターは保持） */
    if (sim->ctrl_id >= 0 && sim->ctrl_id < sim->model->nu) {
        sim->data->ctrl[sim->ctrl_id] = sim->ctrl;
    }
    mj_step(sim->model, sim->data);
    _update_position(sim);
    pthread_mutex_unlock(&sim->lock);
    _pace(sim);
}

/** seconds 秒分のシミュレーションステップを実行する（実時間でもseconds秒進む） */
static void sim_sleep(sim_spikehat_t *sim, double seconds) {
    int steps = (int)(seconds / sim->model->opt.timestep);
    if (steps < 1) steps = 1;
    for (int i = 0; i < steps; i++) {
        sim_step(sim);
    }
}


/* ── 初期化 / 終了 ──────────────────────────────────────────────── */

/**
 * spikehat_open — MuJoCo XMLからシムを初期化する
 *
 * @param device  MuJoCo XML ファイルのパスを渡す（実機のシリアルパスの代わり）
 * @return        sim_spikehat_t * を spikehat_t * にキャストして返す
 */
spikehat_t *spikehat_open(const char *device) {
    /* 環境変数 SPIKEHAT_SIM_XML が設定されていればXMLパスとして使う
     * 未設定の場合は device 引数をそのまま使う（通常は /dev/serial0 が渡される） */
    const char *xml_path = getenv("SPIKEHAT_SIM_XML");
    if (!xml_path || xml_path[0] == '\0') {
        xml_path = device;
    }

    sim_spikehat_t *sim = calloc(1, sizeof(sim_spikehat_t));
    if (!sim) return NULL;

    /* MuJoCo モデルのロード */
    char error[1024] = "";
    sim->model = mj_loadXML(xml_path, NULL, error, sizeof(error));
    if (!sim->model) {
        fprintf(stderr, "sim_spikehat: mj_loadXML failed: %s\n", error);
        free(sim);
        return NULL;
    }
    sim->data = mj_makeData(sim->model);
    if (!sim->data) {
        fprintf(stderr, "sim_spikehat: mj_makeData failed\n");
        mj_deleteModel(sim->model);
        free(sim);
        return NULL;
    }

    /* mj_step（バックグラウンドスレッド）と spikehat_sim_set_ctrl
     * （ビューア側スレッド）が data に同時アクセスするのを防ぐ */
    pthread_mutex_init(&sim->lock, NULL);

    /* 速度スケールの設定
     * 環境変数 SPIKEHAT_SIM_SPEED_SCALE で上書き可能（デフォルト: 1.0 = 実機と同じ時間感覚）
     * 例: SPIKEHAT_SIM_SPEED_SCALE=10.0 で10倍速シミュレーション */
    const char *scale_env = getenv("SPIKEHAT_SIM_SPEED_SCALE");
    sim->speed_scale = (scale_env && scale_env[0] != '\0')
                       ? atof(scale_env) : DEFAULT_SPEED_SCALE;

    /* motor_joint のインデックスを検索（なければ -1 でモーター機能無効） */
    sim->joint_id = mj_name2id(sim->model, mjOBJ_JOINT, "motor_joint");
    if (sim->joint_id < 0) {
        fprintf(stderr, "[sim] motor_joint not found: motor functions disabled\n");
        sim->qpos_adr = -1;
    } else {
        sim->qpos_adr = sim->model->jnt_qposadr[sim->joint_id];
    }
    /* motor_joint に対応するアクチュエーターIDを検索 */
    sim->ctrl_id = -1;
    for (int i = 0; i < sim->model->nu; i++) {
        if (sim->model->actuator_trnid[i * 2] == sim->joint_id) {
            sim->ctrl_id = i;
            break;
        }
    }
    fprintf(stderr, "[sim] motor ctrl_id=%d\n", sim->ctrl_id);

    /* 初期状態を計算 */
    mj_forward(sim->model, sim->data);
    sim->prev_qpos    = (sim->qpos_adr >= 0) ? sim->data->qpos[sim->qpos_adr] : 0.0;
    sim->position_deg = 0.0;
    sim->ctrl         = 0.0;
    sim->running      = 1;
    sim->color_site_id = mj_name2id(sim->model, mjOBJ_SITE, "color_site");
    fprintf(stderr, "[sim] color_site id=%d\n", sim->color_site_id);
    /* distance_site を検索（sonar_site もフォールバックとして試みる） */
    sim->distance_site_id = mj_name2id(sim->model, mjOBJ_SITE, "distance_site");
    if (sim->distance_site_id < 0)
        sim->distance_site_id = mj_name2id(sim->model, mjOBJ_SITE, "sonar_site");
    fprintf(stderr, "[sim] distance_site id=%d\n", sim->distance_site_id);
    /* force_site を検索 */
    sim->force_site_id = mj_name2id(sim->model, mjOBJ_SITE, "force_site");
    /* touch センサーを検索 */
    sim->force_sensor_id = mj_name2id(sim->model, mjOBJ_SENSOR, "force_touch");
    fprintf(stderr, "[sim] force_site id=%d  force_touch id=%d\n",
            sim->force_site_id, sim->force_sensor_id);

    fprintf(stderr, "[sim] spikehat_open: %s (timestep=%.4f, speed_scale=%.1f)\n",
            xml_path, sim->model->opt.timestep, sim->speed_scale);

    return (spikehat_t *)sim;
}

void spikehat_close(spikehat_t *hat) {
    if (!hat) return;
    sim_spikehat_t *sim = (sim_spikehat_t *)hat;

    /* 全モーターを停止 */
    sim->ctrl = 0.0;
    sim_step(sim);

    mj_deleteData(sim->data);
    mj_deleteModel(sim->model);
    pthread_mutex_destroy(&sim->lock);
    free(sim);

    fprintf(stderr, "[sim] spikehat_close: 終了\n");
}


/* ── ポート設定 ─────────────────────────────────────────────────── */

int spikehat_port_config(spikehat_t *hat, int port, spikehat_device_t type) {
    if (!hat || port < 0 || port >= SPIKEHAT_MAX_PORTS) return -1;
    sim_spikehat_t *sim = (sim_spikehat_t *)hat;
    sim->ports[port].device  = type;
    sim->ports[port].valid   = 0;
    sim->ports[port].nvalues = 0;
    fprintf(stderr, "[sim] port_config(port=%d, device=%d)\n", port, type);
    return 0;
}


/* ── モーター制御 ───────────────────────────────────────────────── */

int spikehat_motor_pwm(spikehat_t *hat, int port, float power) {
    if (!hat) return -1;
    sim_spikehat_t *sim = (sim_spikehat_t *)hat;
    if (power >  1.0f) power =  1.0f;
    if (power < -1.0f) power = -1.0f;
    sim->ctrl = (double)power;
    sim_step(sim);
    return 0;
}

int spikehat_motor_start(spikehat_t *hat, int port, int speed) {
    if (!hat) return -1;
    sim_spikehat_t *sim = (sim_spikehat_t *)hat;
    sim->ctrl = speed * SPEED_TO_CTRL;
    sim_step(sim);
    return 0;
}

int spikehat_motor_stop(spikehat_t *hat, int port) {
    if (!hat) return -1;
    sim_spikehat_t *sim = (sim_spikehat_t *)hat;
    sim->ctrl = 0.0;
    sim_step(sim);
    return 0;
}

int spikehat_motor_coast(spikehat_t *hat, int port) {
    return spikehat_motor_stop(hat, port);
}

int spikehat_motor_run_for_seconds(spikehat_t *hat, int port,
                                   float seconds, int speed) {
    if (!hat) return -1;
    sim_spikehat_t *sim = (sim_spikehat_t *)hat;
    sim->ctrl = speed * SPEED_TO_CTRL;
    sim_sleep(sim, (double)seconds);
    sim->ctrl = 0.0;
    sim_step(sim);
    return 0;
}

int spikehat_motor_run_for_degrees(spikehat_t *hat, int port,
                                   int degrees, int speed) {
    if (!hat || speed == 0) return -1;
    sim_spikehat_t *sim = (sim_spikehat_t *)hat;
    if (sim->qpos_adr < 0 || sim->ctrl_id < 0) return -1;

    /* gear の符号を取得して回転方向を決める
     * actuator_gear は6要素配列で、最初の要素がスカラーgear値 */
    double gear = sim->model->actuator_gear[sim->ctrl_id * 6];
    double direction = (gear >= 0) ? 1.0 : -1.0;

    /* 目標角度を計算（gear符号に追従） */
    double cur    = sim->position_deg;
    double target = cur + direction * (double)degrees;

    /* 速度の符号で方向を決める */
    int actual_speed = (degrees >= 0) ? abs(speed) : -abs(speed);
    sim->ctrl = actual_speed * SPEED_TO_CTRL;

    double stop_at = target;

    /* stop_at に達するまでステップ実行 */
    int max_steps = (int)(10.0 / sim->model->opt.timestep);
    for (int i = 0; i < max_steps; i++) {
        sim_step(sim);
        double pos = sim->position_deg;
        if (degrees >= 0 && direction * (pos - stop_at) >= 0.0) break;
        if (degrees <  0 && direction * (pos - stop_at) <= 0.0) break;
    }

    /* ブレーキ: ctrl=0 にして慣性が収まるまで少し待つ */
    sim->ctrl = 0.0;
    int settle = (int)(0.5 / sim->model->opt.timestep);
    for (int i = 0; i < settle; i++) {
        sim_step(sim);
    }
    return 0;
}

int spikehat_motor_run_to_position(spikehat_t *hat, int port,
                                   int position_deg, int speed) {
    if (!hat || speed == 0) return -1;
    sim_spikehat_t *sim = (sim_spikehat_t *)hat;
    if (sim->qpos_adr < 0 || sim->ctrl_id < 0) return -1;

    /* gear の符号を取得して回転方向を決める */
    double gear = sim->model->actuator_gear[sim->ctrl_id * 6];
    double direction = (gear >= 0) ? 1.0 : -1.0;

    /* 目標位置までの移動量（degrees相当）を direction を考慮して算出 */
    double cur           = sim->position_deg;
    double target        = (double)position_deg;
    double degrees_equiv = (target - cur) * direction;

    /* 速度の符号で方向を決める */
    int actual_speed = (degrees_equiv >= 0) ? abs(speed) : -abs(speed);
    sim->ctrl = actual_speed * SPEED_TO_CTRL;

    /* target に達するまでステップ実行 */
    int max_steps = (int)(10.0 / sim->model->opt.timestep);
    for (int i = 0; i < max_steps; i++) {
        sim_step(sim);
        double pos = sim->position_deg;
        if (degrees_equiv >= 0 && direction * (pos - target) >= 0.0) break;
        if (degrees_equiv <  0 && direction * (pos - target) <= 0.0) break;
    }

    /* ブレーキ: ctrl=0 にして慣性が収まるまで少し待つ */
    sim->ctrl = 0.0;
    int settle = (int)(0.5 / sim->model->opt.timestep);
    for (int i = 0; i < settle; i++) {
        sim_step(sim);
    }
    return 0;
}

int spikehat_motor_get_speed(spikehat_t *hat, int port, int *speed) {
    if (!hat || !speed) return -1;
    sim_spikehat_t *sim = (sim_spikehat_t *)hat;
    *speed = (int)round(sim->ctrl / SPEED_TO_CTRL);
    return 0;
}

int spikehat_motor_get_position(spikehat_t *hat, int port, int *degrees) {
    if (!hat || !degrees) return -1;
    sim_spikehat_t *sim = (sim_spikehat_t *)hat;
    if (sim->qpos_adr < 0) return -1;  /* モーターなし */
    *degrees = (int)round(sim->position_deg);
    return 0;
}


/* ── センサー読み取り ────────────────────────────────────────────── */

int spikehat_distance_read(spikehat_t *hat, int port, int *mm) {
    if (!hat || !mm) return -1;
    sim_spikehat_t *sim = (sim_spikehat_t *)hat;
    if (sim->distance_site_id < 0) { *mm = DIST_INVALID; return 0; }

    /* distance_site の現在位置 */
    double *sp = &sim->data->site_xpos[sim->distance_site_id * 3];

    /* site_xmat の Y軸正方向が前方 */
    double *xmat      = &sim->data->site_xmat[sim->distance_site_id * 9];
    double forward[3] = { xmat[1], xmat[4], xmat[7] };

    /* distance_site が属する body を除外してレイキャスト */
    int site_bodyid = sim->model->site_bodyid[sim->distance_site_id];
    int geomid_out[1] = { -1 };
    mjtNum normal[3];
    mjtNum dist = mj_ray(sim->model, sim->data,
                         sp, forward, NULL, 1, site_bodyid,
                         geomid_out, normal);

    if (dist < 0 || dist > DIST_MAX_M) {
        *mm = DIST_INVALID;
    } else {
        *mm = (int)round(dist * 1000.0);
    }
    return 0;
}

static void rgb_to_hsv(double r, double g, double b,
                       int *hue, int *sat, int *val) {
    double max = r > g ? (r > b ? r : b) : (g > b ? g : b);
    double min = r < g ? (r < b ? r : b) : (g < b ? g : b);
    double delta = max - min;
    *val = (int)round(max * 1000.0);
    *sat = (max > 0.0) ? (int)round((delta / max) * 1000.0) : 0;
    if (delta < 1e-6) { *hue = 0; return; }
    double h;
    if (max == r)      h = 60.0 * fmod((g - b) / delta, 6.0);
    else if (max == g) h = 60.0 * ((b - r) / delta + 2.0);
    else               h = 60.0 * ((r - g) / delta + 4.0);
    if (h < 0.0) h += 360.0;
    *hue = (int)round(h);
}

int spikehat_color_read_hsv(spikehat_t *hat, int port,
                             int *hue, int *sat, int *val) {
    if (!hat || !hue || !sat || !val) return -1;
    sim_spikehat_t *sim = (sim_spikehat_t *)hat;
    if (sim->color_site_id < 0) { *hue = *sat = *val = 0; return 0; }

    /* color_site の現在位置 */
    double *sp = &sim->data->site_xpos[sim->color_site_id * 3];

    /* 真下方向 */
    double down[3] = { 0.0, 0.0, -1.0 };

    /* color_site が属する body を除外してレイキャスト */
    int site_bodyid = sim->model->site_bodyid[sim->color_site_id];
    int geomid_out[1] = { -1 };
    mjtNum normal[3];
    mjtNum dist = mj_ray(sim->model, sim->data,
                         sp, down, NULL, 1, site_bodyid, geomid_out, normal);

    if (dist < 0 || geomid_out[0] < 0) {
        *hue = *sat = *val = 0;
        return 0;
    }

    /* ヒットしたgeomのRGBAからHSVを計算 */
    int gid = geomid_out[0];
    float *rgba = &sim->model->geom_rgba[gid * 4];
    rgb_to_hsv((double)rgba[0], (double)rgba[1], (double)rgba[2],
               hue, sat, val);
    return 0;
}

int spikehat_color_read_rgb(spikehat_t *hat, int port,
                             int *r, int *g, int *b) {
    if (!hat || !r || !g || !b) return -1;
    sim_spikehat_t *sim = (sim_spikehat_t *)hat;
    if (sim->color_site_id < 0) { *r = *g = *b = 0; return 0; }

    /* color_site の現在位置 */
    double *sp = &sim->data->site_xpos[sim->color_site_id * 3];

    /* 真下方向 */
    double down[3] = { 0.0, 0.0, -1.0 };

    /* レイキャスト */
    int site_bodyid = sim->model->site_bodyid[sim->color_site_id];
    int geomid_out[1] = { -1 };
    mjtNum normal[3];
    mjtNum dist = mj_ray(sim->model, sim->data,
                         sp, down, NULL, 1, site_bodyid,
                         geomid_out, normal);

    if (dist < 0 || geomid_out[0] < 0) {
        *r = *g = *b = 0;
        return 0;
    }

    /* geomのRGBAから RGB (0〜255) に変換 */
    float *rgba = &sim->model->geom_rgba[geomid_out[0] * 4];
    *r = (int)round(rgba[0] * 255.0f);
    *g = (int)round(rgba[1] * 255.0f);
    *b = (int)round(rgba[2] * 255.0f);
    return 0;
}

int spikehat_force_read(spikehat_t *hat, int port,
                        int *force, int *pressed) {
    if (!hat || !force || !pressed) return -1;
    sim_spikehat_t *sim = (sim_spikehat_t *)hat;
    if (sim->force_sensor_id < 0) { *force = 0; *pressed = 0; return 0; }

    /* touch センサーの値を取得（接触力 [N]） */
    int adr = sim->model->sensor_adr[sim->force_sensor_id];
    double f = sim->data->sensordata[adr];

    *force   = (int)round(f);
    *pressed = (f > 0.1) ? 1 : 0;  /* 0.1N以上で押下判定（実機に合わせる） */
    return 0;
}

int spikehat_force_is_pressed(spikehat_t *hat, int port, int *pressed) {
    int force = 0;
    if (spikehat_force_read(hat, port, &force, pressed) != 0) return -1;
    return 0;
}

int spikehat_force_get_force(spikehat_t *hat, int port, int *force) {
    int pressed = 0;
    if (spikehat_force_read(hat, port, force, &pressed) != 0) return -1;
    return 0;
}

void spikehat_sleep(spikehat_t *hat, float seconds) {
    if (!hat) return;
    sim_spikehat_t *sim = (sim_spikehat_t *)hat;
    sim_sleep(sim, (double)seconds);
}

/* ── シム専用拡張API ──────────────────────────────────────────────── */

int spikehat_sim_set_ctrl(spikehat_t *hat, int actuator_id, double val) {
    if (!hat || actuator_id < 0) return -1;
    sim_spikehat_t *sim = (sim_spikehat_t *)hat;
    if (actuator_id >= sim->model->nu) return -1;
    pthread_mutex_lock(&sim->lock);
    sim->data->ctrl[actuator_id] = val;
    pthread_mutex_unlock(&sim->lock);
    return 0;
}

int spikehat_sim_get_qpos(spikehat_t *hat, int qpos_adr, double *out) {
    if (!hat || !out || qpos_adr < 0) return -1;
    sim_spikehat_t *sim = (sim_spikehat_t *)hat;
    if (qpos_adr >= sim->model->nq) return -1;
    pthread_mutex_lock(&sim->lock);
    *out = sim->data->qpos[qpos_adr];
    pthread_mutex_unlock(&sim->lock);
    return 0;
}

void *spikehat_sim_get_model(spikehat_t *hat) {
    if (!hat) return NULL;
    return ((sim_spikehat_t *)hat)->model;
}

void *spikehat_sim_get_data(spikehat_t *hat) {
    if (!hat) return NULL;
    return ((sim_spikehat_t *)hat)->data;
}

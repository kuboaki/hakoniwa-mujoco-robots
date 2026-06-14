#ifndef SPIKEHAT_SIM_H
#define SPIKEHAT_SIM_H

/**
 * spikehat_sim.h — libspikehat_sim 専用の拡張API
 *
 * このヘッダーは libspikehat_sim でのみ使用する。
 * 実機の libspikehat には存在しない。
 * テストや自動化スクリプトからシムの内部状態を操作するために使う。
 */

#include "robots/spikehat/spikehat.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * spikehat_sim_set_ctrl — アクチュエーターのctrl値を直接設定する
 *
 * シム内部の data->ctrl[actuator_id] を設定する。
 * スライダーや外部アクチュエーターをテストから操作する際に使用する。
 *
 * @param hat          SpikeHat インスタンス
 * @param actuator_id  アクチュエーターのID（mj_name2id で取得）
 * @param val          設定する値
 * @return 0: 成功, -1: 失敗
 */
int spikehat_sim_set_ctrl(spikehat_t *hat, int actuator_id, double val);

/**
 * spikehat_sim_get_qpos — 関節位置 data->qpos[qpos_adr] を取得する
 *
 * ビューア表示用に、実シミュレーション側の関節位置を取得する際に使用する。
 *
 * @param hat       SpikeHat インスタンス
 * @param qpos_adr  qposインデックス（model.jnt_qposadr で取得）
 * @param out       取得した値の格納先
 * @return 0: 成功, -1: 失敗
 */
int spikehat_sim_get_qpos(spikehat_t *hat, int qpos_adr, double *out);

/**
 * spikehat_sim_get_model — MuJoCo モデルのポインタを返す
 * @return mjModel* （実機版では NULL）
 */
void *spikehat_sim_get_model(spikehat_t *hat);

/**
 * spikehat_sim_get_data — MuJoCo データのポインタを返す
 * @return mjData* （実機版では NULL）
 */
void *spikehat_sim_get_data(spikehat_t *hat);

#ifdef __cplusplus
}
#endif

#endif /* SPIKEHAT_SIM_H */

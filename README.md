# wrwc_ros
- つくチャレ用のパッケージ郡(テスト用)

## rwrc_driver
- ドライバ用パッケージ

## rwrc_simulator
- シミュレータ用パッケージ
- TODO: simple_icart_middle.urdfの寸法を正確に
- TODO: ROSの時刻がおかしい？(未来の時刻を参照しようとしてるエラーが出る)
    - walltime→timeに変換すれば良い？
- getSensors()ノード仕様
    - とりあえず最新の値を格納(ROS topicのまま)
    - 一定周期でgetSensors関数の実行、値の更新
        - jointをエンコーダ値に変換
        - ジャイロからヨー角変化を取得
        - odom計算
        - tf出力
    - 点群座標変換(robot基準、現在地図座標系基準)
        - getSensorsで得たtfを用いる
        - scan型なのでとりあえずはtopicとして出力(debug用)
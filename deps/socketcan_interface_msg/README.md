Jetson の formula_ws に `socketcan_interface_msg` が無いときだけ、このパッケージをワークスペースに取り込む。
パッケージルートで `ln -s $(pwd)/deps/socketcan_interface_msg ../socketcan_interface_msg` を実行して再ビルドする。

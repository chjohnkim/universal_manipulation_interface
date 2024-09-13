cd /home/amiga-bak/johnkim/universal_manipulation_interface
sudo chmod -R 777 /dev/bus/usb
sudo chmod -R 666 /dev/ttyUSB0
/home/amiga-bak/miniconda3/envs/umi/bin/python eval_real.py --robot_config=example/eval_robots_config.yaml -i ./ckpts/pruner_inverse_multipick.ckpt -o data/eval_pruner_inverse_multipick
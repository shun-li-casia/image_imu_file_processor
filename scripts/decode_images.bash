#!/usr/bin/zsh

######################################################################
# @author      : ShunLi
# @file        : decode_images
# @created     : 星期二 8月 15, 2023 09:16:43 CST
#
# @description : 
######################################################################

gnome-terminal --window  -e 'zsh -c "source ~/.zshrc; roscore; exec zsh"' \
--tab -e 'zsh -c "source ~/.zshrc; sleep 1; rosrun image_imu_processor test_img_decoder --save_img=0 --save_video=1 --jump=0 --output_path=. --bin_path=cam_ch_00.bin;  exec zsh"' \
--tab -e 'zsh -c "source ~/.zshrc; sleep 1; rosrun image_imu_processor test_img_decoder --save_img=0 --save_video=1 --jump=0 --output_path=. --bin_path=cam_ch_01.bin;  exec zsh"' \
--tab -e 'zsh -c "source ~/.zshrc; sleep 1; rosrun image_imu_processor test_img_decoder --save_img=0 --save_video=1 --jump=0 --output_path=. --bin_path=cam_ch_02.bin;  exec zsh"' \
--tab -e 'zsh -c "source ~/.zshrc; sleep 1; rosrun image_imu_processor test_img_decoder --save_img=0 --save_video=1 --jump=0 --output_path=. --bin_path=cam_ch_03.bin;  exec zsh"' \
--tab -e 'zsh -c "source ~/.zshrc; sleep 1; rosrun image_imu_processor test_img_decoder --save_img=0 --save_video=1 --jump=0 --output_path=. --bin_path=cam_ch_04.bin;  exec zsh"' \
--tab -e 'zsh -c "source ~/.zshrc; sleep 1; rosrun image_imu_processor test_img_decoder --save_img=0 --save_video=1 --jump=0 --output_path=. --bin_path=cam_ch_05.bin;  exec zsh"' \
--tab -e 'zsh -c "source ~/.zshrc; sleep 1; rosrun image_imu_processor test_img_decoder --save_img=0 --save_video=1 --jump=0 --output_path=. --bin_path=cam_ch_06.bin;  exec zsh"' \
--tab -e 'zsh -c "source ~/.zshrc; sleep 1; rosrun image_imu_processor test_img_decoder --save_img=0 --save_video=1 --jump=0 --output_path=. --bin_path=cam_ch_07.bin;  exec zsh"' \
--tab -e 'zsh -c "source ~/.zshrc; sleep 1; rosrun image_imu_processor test_img_decoder --save_img=0 --save_video=1 --jump=0 --output_path=. --bin_path=cam_ch_08.bin;  exec zsh"' \
--tab -e 'zsh -c "source ~/.zshrc; sleep 1; rosrun image_imu_processor test_img_decoder --save_img=0 --save_video=1 --jump=0 --output_path=. --bin_path=cam_ch_09.bin;  exec zsh"' \
--tab -e 'zsh -c "source ~/.zshrc; sleep 1; rosrun image_imu_processor test_img_decoder --save_img=0 --save_video=1 --jump=0 --output_path=. --bin_path=cam_ch_10.bin;  exec zsh"' \

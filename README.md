# nmpc-mav
Nonlinear model predictive control for leader-follower quadrotors with observability constraint. This simulation takes the range-based multi-robot model and Acados MPC library, to compute the optimal control inputs for better localization and control.

## Installation of Acados
 Install acados according to https://github.com/acados/acados. The steps shoud be:

	$ git clone https://github.com/acados/acados.git
	$ git submodule update --recursive --init
	$ mkdir -p build
	$ cd build
	$ cmake ..
	# cmake -DACADOS_WITH_QPOASES=ON ..
	# add more optional arguments e.g. -DACADOS_WITH_OSQP=OFF/ON -DACADOS_INSTALL_DIR=<path_to_acados_installation_folder> above
	$ make install

## Create Python env
This procedure is according to https://docs.acados.org/python_interface/index.html, which should be

	$ sudo apt install virtualenv
	$ virtualenv envmpc --python=/usr/bin/python3.7 #sudo rm -rf envmpc
	$ source envmpc/bin/activate
	$ pip3 install -e /home/lss/acados/interfaces/acados_template
	$ export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"/home/lss/acados/lib"
	$ export ACADOS_SOURCE_DIR="/home/lss/acados"
	Hint: you can add these two lines to your .bashrc/.zshrc.

## Run the nmpc-mav code
	$ source envmpc/bin/activate
	$ export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"/home/lss/acados/lib"
	$ export ACADOS_SOURCE_DIR="/home/lss/acados"
	$ git clone https://github.com/shushuai3/nmpc-mav.git
	$ cd nmpc-mav
	$ python main.py
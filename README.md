# Monte Carlo Localization

## Instructions

1. Create a catkin workspace that will contain the code inside the home

```bash
cd ~
mkdir -p catkin_ws/src
cd catkin_ws
catkin build
```

2. Clone this repository inside the catkin_ws/src folder

```bash
cd src
git clone https://github.com/mhteixeira/monte-carlo-localization.git
```

3. Build the package

```bash
cd ~/catkin_ws
catkin build
```

4. Include the following line inside the ~/.bashrc file (using e.g. 'nano ~/.bashrc')

``` bash 
source $HOME/catkin_ws/devel/setup.bash
```

and then do:

``` bash 
source ~/.bashrc
```

4. Run the launch file

``` bash 
roslaunch mcl_python load_mcl.launch
```

## Tarefas

1. Conseguir o laser para o Pioneer-3DX

2. Criar o mapa do ambiente com o Gmapping

3. Criar a estrutura geral da implementação
    1. Criar a classe Particle
    2. Criar a classe ParticleFilter
        - Lê o mapa (__init__)
        - Inicializa de forma aleatória as partículas (random)
            - Escolhe uma pose aleatória (x, y, theta)
            - Vê se ela está dentro do mapa
            - Se não estiver dentro do mapa, procuro outro lugar
        - Atualizar as partículas (Algorithm MCL)
            - Reescolher de forma aleatória numa primeira implementação
        - Printa as partículas (usando matplotlib)

4. Implementar o algoritmo MCL
    1. Criar e testar o Odometry Motion Model
    2. Criar e testar o Likelihood Field Algorithm
    3. Criar e testar o Low Variance Sampler
    4. Integrar os algoritmos implementados na estrutura pré-definida
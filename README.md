# Monte Carlo Localization

## Instructions

1. Rodar o roscore

``` bash
roscore
```

2. Para rodar o rosbag

``` bash
cd bagfiles
rosbag play -l -r 0.5 2023-05-05-10-05-00.bag
```

3. Build dos pacotes
``` bash 
cd ~/monte-carlo-localization
catkin build
```

3. Rodar o arquivo python
``` bash 
rosrun nome_do_pacote nome_do_arquivo.py
```

e.g.

``` bash 
rosrun reading_odometry listener.py
```


## Tarefas

1. Conseguir o laser para o Pioneer-3DX: Bernardes

2. Criar o mapa do ambiente com o Gmapping ([Mapa com o Pioneer-3DX](http://wiki.ros.org/p2os-purdue/Tutorials/GMapping%20With%20Pioneer-3dx))

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
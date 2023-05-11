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

3. Rodar o arquivo python
``` bash 
rosrun nome_do_pacote nome_do_arquivo.py
```


## Tarefas

1. Conseguir o laser para o Pioneer-3DX

2. Criar o mapa do ambiente com o Gmapping ([Mapa com o Pioneer-3DX](http://wiki.ros.org/p2os-purdue/Tutorials/GMapping%20With%20Pioneer-3dx))

3. Criar node que publica o mapa para o RViz

4. Criar o node do Monte Carlo
    1. Ler dados da odometria e laser
    2. Implementar uma classe de Particle Filter

5. Publicar as partículas para o RViz
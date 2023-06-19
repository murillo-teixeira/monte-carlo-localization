# Monte Carlo Localization

## Repositório

O código completo, com todas as bags de teste pode ser encontrado [aqui](https://github.com/mhteixeira/monte-carlo-localization).

## Considerações sobre o código

Os parâmetros da execução do programa são controlados alterando-se os arquivos ```load_mcl.launch```, onde são escolhidas as bags, e ```config.yaml```, em que encontram-se o número de partículas, frequência do mcl_node, forma de plot, dentre outros.

O arquivo onde o node é definido se encontra em ```src/mcl_python/mcl_node.py```.

As classes criadas encontram-se em ```src/classes```. E os scripts auxiliares (dentre os quais as microssimulações para teste dos algoritmos) encontram-se em ```src/scripts```

## Instruções para rodar o programa

1. Crie um workspace do catkin:

```bash
cd ~
mkdir -p catkin_ws/src
cd catkin_ws
catkin build
```

2. Incluir esse repositório na pasta catkin_ws/src:

```bash
cd src
git clone https://github.com/mhteixeira/monte-carlo-localization.git
```

3. Fazer o build do pacote:

```bash
cd ~/catkin_ws
catkin build
```

4. Incluir a linha seguinte em ~/.bashrc (using e.g. 'nano ~/.bashrc'):

``` bash 
source $HOME/catkin_ws/devel/setup.bash
```

e depois dar source:

``` bash 
source ~/.bashrc
```

4. Rodar o arquivo .launch do projeto:

``` bash 
roslaunch mcl_python load_mcl.launch
```
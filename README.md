# Monte Carlo Localization

Neste repositório se encontra a implementação do algoritmo Monte Carlo Localization para o robô Pioneer 3DX desenvolvida para a disciplina Sistemas Autónomos do Instituto Superior Técnico - Universidade de Lisboa.

## Resultados

Para avaliar a performance da implementação, foram gravadas ```rosbags``` em 3 diferentes ambientes da Torre Norte do IST e o algoritmo foi executado em modo de pose tracking e global localization e comparado com o pacote amcl, como pode ser visto nas imagens abaixo.

Para mais informações sobre o projeto e resultados, é possível acessar o relatório completo na raiz do repositório.

<table>
  <tr>
     <td>Pose tracking</td>
    <td>Global localization</td>
  </tr>
  <tr>
    <td><img src="images/all_local.png" width=270></td>
    <td><img src="images/all_global.png" width=270></td>
  </tr>
 </table>

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
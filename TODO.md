SIGKILL qunado os hosts não estão todos conectados?? ver melhor !!!

se for para avançar com o consumo é preciso ter em conta também o consumo de cada porta, fazer funcao extendendo NodeEnergyModel, ver exemplos parecidos. (assegurar que o parser lê a nova opção desenvolvida)

-----------------------

mudar equalCostPaths para um 'map' e dar refactoring ao código 

ver a questão do utilização dos links antes de atribuir fluxos


URGENTE: ver a questão da atualização do custo dos caminhos!! rever lógica toda

ideia, não guardar os pesos, e só recalcular e guardar o peso do caminho quando for necessário, ou seja, quando se for procurar o caminho mais curto ?????
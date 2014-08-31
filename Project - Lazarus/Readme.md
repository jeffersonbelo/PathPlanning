Alguns detalhes analisados:

	1 - GetBestPath(st_x,st_y,tg_x,tg_y: double; var traj: TTrajectory; var obs: array of TRoundObstacle; num_obs: integer);  - Linha 609
		Esse método recebe os pontos X e Y do ponto inicial e final, a trajetória gerada, os obstáculos presente no mapa [x,y,r] e o número deles.
		O mesmo é responsável em verificar se há ou não obstáculos presente na trajetória passada por parâmetro e tomar as medidas necessárias para lidar com elas. 

			Inicialmente o mesmo chama a função: ObstacleInSegment(st_x,st_y,tg_x,tg_y: double; var obs: array of TRoundObstacle; num_obs: integer): boolean; - Linha 595, 
			Este método vai verificar se há obstáculos presente no segmento passado, retornando TRUE ou FALSE. 
					
				Para isso, o mesmo inicialmente chama a função Dist(x,y):double [RobotMainFuncions-653] para verificar se a distância entre os pontos passados é menor que o raio de algum obstáculo presente no grafo, utilizando a seguinte formula: Dist = sqrt(x²+y²) se for menor é porque colidiu e há obstáculos e deve ser retornado TRUE. Caso o valor do raio seja maior, quer dizer que não foi detectado nenhum obstáculo direto no segmento e deverá ser avaliada agora se há alguma interseção com os obstáculos do grafo. 
					
				Para verificar se existe interseção é chamado o método CalcIntersectionPoint(x1,y1,x2,y2: double; var vx,vy: double; obs: TRoundObstacle): boolean; - Linha 546
				O mesmo, através de vários cálculos geométricos considerando os pontos passados por parâmetro e os obstáculos, verifica se intersectam ou não. Se for localizada a interseção, significará que há obstáculos presentes no segmento e deverá ser retornado TRUE, caso não localize, será retornado FALSE
					

			De acordo com a resposta retornada pelo método ObstacleInSegment é tomada as medidas necessárias para lidar com a situação, dentre elas estão:
			
			Se NÃO houver nenhum obstáculo no segmento:		
				Será executado a procedure BuildSimpleTrajectory(var traj: TTrajectory; sx,sy,tx,ty: double) - Linha 582.
				A mesma tem como objetivo construir uma trajetória simples, interligando diretamente o ponto corrente ao ponto objetivo, ambos esses recebidos por parâmetro pela procedure. (Target.X-Start.X, Target.Y-Start.y)
			
			Se houver obstáculos presentes:
				Será chamado a procedure  AStarGetBestPath(st_x, st_y, tg_x, tg_y, traj, obs, num_obs); - Linha 491, 
				Para lidar com o planejamento do caminho necessário para o desvio dos obstáculos presentes no segmento e alcance do objetivo.
	
	[Inicialmente acredito que não será necessário adaptar essa parte do código]

				
	2 - AStarGetBestPath(st_x, st_y, tg_x, tg_y, traj, obs, num_obs); - Linha 491.
		Esse método recebe os pontos X e Y do ponto inicial e final, a trajetória gerada, os obstáculos presente no mapa [x,y,r] e o número deles. O mesmo é responsável em executar todas as ações necessárias para que seja planejada e executada uma trajetória que interligue o ponto incial ao ponto objetivo, desviando os obstáculos existentes. Para que isso seja feito é executado a procedure ...
		
		AStarClear(var iMap: TAStarMap); - Linha 162,
			Esse é responsável em limpar o mapa (caso exista alguma informação nele) e setar todos os nós para virgens (Não visitados ainda), bem como construir uma parede para não precisar se importar com os limites do campo e simplificar a execução do planejador, por fim ele limpa o contador da pilha. 
							
			Em seguida é feita alguma transformação geometrica ( TIRAR DÚVIDAS COM O PROFESSOR ) 
			Linha 500 (AStar) bem como perguntar o que é esse traj passado por parametro, são todas as trajetórias possiveis, recebidas pelo decomposição por celulas (contendo a informação geral do grafo) ou alguma trajetoria especifica (contendo informação especifica de uma iteracao ), perguntar tambem a linha 523-525 em seguida é executada a procedure ...
				
						
	3 - AStarGo(var iMap: TAStarMap); - Linha 414
		A mesma é a responsável em executar todas as ações necessária para que o planejador A* funcione. Imediatamente é já é executada a procedure ...
				
		AStarCheckBoundaries(var Map: TAStarMap); - Linha 401
		A mesma tem o objetivo  checar alguns limites do campo e verificar se o mesmo está todo ok;
		
			Ele executa duas vezes a procedure AStarCheckBoundary(var pnt: TGridCoord; var Map: TAStarMap); 
			Que tem como objetivo verificar se o ponto passado por parametro está ou não dentro de um obstáculo e se sim, move o mesmo para um espaço aberto.
			( Perguntar ao professor como é esse comportamente ) 
			Em seguinda, é executa a procedure ...
						
	4 - AStarInit(var iMap: TAStarMap);    - Linha 407	
		A mesma executa a procedure AddToAStarList( var Map: TAStarMap; Pnt: TGridCoord); Linha 314. 
			Essa tem como objetivo adciona o ponto  inicial do mapa a lista aberta, para isso a mesma: 
						
						Incrementa o contador e define o estado passado por parametro com o valor 3 ( valor que informa ao programa que a celula está na lista aberta ), 
						
						em seguida esse ponto é inserida na pilha e executada a procedure UpdateHeapPositionByPromotion(Map, idx); - Linha 225, que verificará se haverá alguma atualização (promoção) dos pais (Bps) presente na pilha. 
						
						Logo em seguida, quando finaliza a promoção, é definido o H do ponto inicial, através da função CalcH(var Map: TAStarMap; Pi, Pf: TGridCoord): integer;  inline; - Linha 333.
				
		Após incluir o ponto inicial na lista aberta e definir seu H o código entra em um ciclo de repetição que executa:
						
	5 - AStarStep(var iMap: TAStarMap); - Linha 338
		Responsável em executar o passo a passo de busca do A*
			Inicialmente é incrementado as variaveis de iteração e contadoras de pilha, posteriormente é executada a procedure RemoveBestFromAStarList(var iMap: TAStarMap; out Pnt: TGridCoord); - Linha 297
				
				É incrementado o contador de remoção
			
				É retornado o primeiro nó, move o ultimo nó para a primeira posicao e decresce o tamanho do array. 
			
			Logo após é executado a procedure UpdateHeapPositionByDemotion(var Map: TAStarMap; idx: integer); - Linha 249.
			A mesma tem como objetivo rebaixar os parents antigos, caso não encontre um bp melhor.  
					
		Em seguida é colocado o ponto atual na lista fechada, definindo o mesmo com o numero 2, e é definido que a vizinha terá 8 nós. 
		
		Em seguida, essa vizinhança é avançada, definindo os novos 8 analisados e é executado alguns blocos CASES. 
					
			Caso o novo ponto já esteja na lista fechada, despreze esse. 
						
			Caso o nó seja virgem, é setado o seu pai e é calculado o seu G e H e em seguida incluido na lista aberta - Linha 314.
					
			Caso o nó esteja na lista aberta, será setado o seu G e verificará se o G atual é menor que G de um nó já presente na aberta, se for, é trocado o pai do nó para o nó atual e recalcula e atualiza as posições na pilha.
						
					Após colocar o target na lista fechada é localizado o objetivo e o laço é interrompido a outra condição de parada é se o contador da pilha chegar a 0, quer dizer que não existe solução.

	6 - Após finalizar esse LOOP, é executada a ultima parte do código,o AStarBuildTrajectory(st_x,st_y,tg_x,tg_y, AStarMap, traj); - Linha 435
	Esse tem como objetivo executar a trajetória localizada ( PEDIR EXPLICAÇÃO AO PROFESSOR ) 
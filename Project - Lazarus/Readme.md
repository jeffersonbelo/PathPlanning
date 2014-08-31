Alguns detalhes analisados:

	1 - GetBestPath(st_x,st_y,tg_x,tg_y: double; var traj: TTrajectory; var obs: array of TRoundObstacle; num_obs: integer);  - Linha 609
		Esse método recebe os pontos X e Y do ponto inicial e final, a trajetória gerada, os obstáculos presente no mapa [x,y,r] e o número deles

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

				
	2 - AStarGetBestPath(st_x, st_y, tg_x, tg_y, traj, obs, num_obs);
	
	
	
	
	
	
	
	
	3 - AStarClear
	4 - AStarGo
	5 - AStarCheckBoudaries
	6 - StarInit
	7 - AddToAStarList
	8 - AStarStep

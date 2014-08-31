Alguns detalhes:

	1 - GetBestPath(st_x,st_y,tg_x,tg_y: double; var traj: TTrajectory; var obs: array of TRoundObstacle; num_obs: integer);  - Linha 609
		Esse método recebe os pontos X e Y do ponto inicial e final, a trajetória gerada, os obstáculos presente no mapa [x,y,r] e o número deles

			Inicialmente o método chama a função: ObstacleInSegment(st_x,st_y,tg_x,tg_y: double; var obs: array of TRoundObstacle; num_obs: integer): boolean; - Linha 595, 
				Este método vai verificar se há obstáculos presente no segmento passado. Caso tenha, retorna TRUE, caso não FALSE
		
				Se NÃO houver nenhum obstáculo no segmento:		
					Será construido uma trajetória simples, ligando diretamente o ponto atual ao ponto objetivo (Target.X-Start.X, Target.Y-Start.y) - Linha 616
				Se houver obstáculos presentes:
					Será chamado o método: AStarGetBestPath(st_x, st_y, tg_x, tg_y, traj, obs, num_obs); para lidar com o planejamento do caminho - Linha 620

	2 - AStarGetBestPath
	3 - AStarClear
	4 - AStarGo
	5 - AStarCheckBoudaries
	6 - StarInit
	7 - AddToAStarList
	8 - AStarStep

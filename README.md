Repo contenant les Modèles Théoriques de différentes Thématiques:
- Stratégie
- Simulation 2D

---------------------------------
1. Stratégie:
     Nous travaillons sur l'Algorithme de Passe Général de l'équipe. Cet algorithme permet de trouver des zones de passes bien définies
   en fonction de la position des robots(Adversaires / Coéquipiers / Porteur de Balle) ainsi que de leurs valeurs de vitesse initiales et max respectives
   2 Déclinaisons de notre méthodes ont été réalisées:
     Version avec Heatmap:
       Nous exécutons l'algorithme sur la totalité du terrain de jeux. Les performances de cette version sont bonnes pour un temps d'exécution important
       ![Alt text](img_readme\Strategy\Passe_direct_heatmap_ok.png?raw=true "Situation avec Passe Direct")
        ![Alt text](img_readme\Strategy\Passe_direct_heatmap_nok.png?raw=true "Situation avec Passe Direct Impossible")
        ![Alt text](img_readme\Strategy\Situation_jeu_heatmap.png?raw=true "Situation de jeux plus complexe")
        ![Alt text](img_readme\Strategy\Temps_exec_Heatmap.png?raw=true "Temps d'exécution de l'algorithme en Secondes")
     Version Centrée autour des Teammates: 
       Nous exécutons l'algorithme à partir des positions de chaques teammates. Les performances de cette version sont inférieures à la version précédente, pour un temps d'exécution bien inférieur 
        ![Alt text](img_readme\Strategy\Passe_direct_ok.png?raw=true "Situation avec Passe Direct")
        ![Alt text](img_readme\Strategy\Passe_direct_nok.png?raw=true "Situation avec Passe Direct Impossible")
        ![Alt text](img_readme\Strategy\Situation_jeu.png?raw=true "Situation de jeux plus complexe")

---------------------------------
2. Simulation 2D:
    Nous avons réaliser un petit exemple de test de trajectoire de balle pour améliorer la physique de notre simulation.
    ![Alt text](img_readme\PhysicSim\Bouncing_Sim.png?raw=true "Simulation Tir avec Rebonds")

    ![Alt text](img_readme\PhysicSim\Rolling_Sim.png?raw=true "Simulation Tir avec balle roulant sur le sol")
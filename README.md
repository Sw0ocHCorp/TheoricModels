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
   ![Passe directe possible](https://github.com/user-attachments/assets/e6ccca6f-c44f-48e6-8d3e-4581f2426701)
   ![Passe directe impossible](https://github.com/user-attachments/assets/8d368f76-4668-40f1-be90-114d5ed402f2)
   ![Situation de jeu plus complexe](https://github.com/user-attachments/assets/f001e47e-d86e-4868-8b90-e413255d5b6f)
   ![Temps d'exécution de l'algorithme en secondes](https://github.com/user-attachments/assets/e5f5b47f-45ed-4f3a-b224-a38535569ceb)

     Version Centrée autour des Teammates: 
       Nous exécutons l'algorithme à partir des positions de chaques teammates. Les performances de cette version sont inférieures à la version précédente, pour un temps d'exécution bien inférieur
   ![Passe directe possible](https://github.com/user-attachments/assets/f467a9da-edd0-44cf-a35b-61bf6399292c)
   ![Passe directe impossible](https://github.com/user-attachments/assets/1a2f3cb8-ac7c-4eb5-8f83-d0f44854acd8)
   ![Situation de jeu plus complexe](https://github.com/user-attachments/assets/ccfeb9c5-9dce-4fdc-b169-57673d263804)


---------------------------------
2. Simulation 2D:
    Nous avons réaliser un petit exemple de test de trajectoire de balle pour améliorer la physique de notre simulation.
    ![Simulation Rebonds après Tir](https://github.com/user-attachments/assets/fcd63165-6dce-489e-a5ab-9dd2196dc2f5)
    ![Simulation Ballon roule au sol après Tir](https://github.com/user-attachments/assets/aa17821d-5599-4e99-94dd-4db8280bca06)

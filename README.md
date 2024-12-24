Repo contenant les Modèles Théoriques de différentes Thématiques:
- Stratégie
- Génération de Trajectoire
- Simulation 2D

---------------------------------
**1. Stratégie:**<br />
   Nous travaillons sur l'Algorithme de Passe Général de l'équipe. Cet algorithme permet de trouver des zones de passes bien définies
   en fonction de la position des robots(Adversaires / Coéquipiers / Porteur de Balle) ainsi que de leurs valeurs de vitesse initiales et max respectives mais également leurs orientations courrantes
   et enfin de leurs vitesses de rotation. Grâce à ces derniers ajouts (orientations / vitesses de rotations) nous fiabilisons notre algorithme en se rapprochant de son cas d'usage réel
   
   2 Déclinaisons de notre méthodes ont été réalisées:<br />
    - Version avec Heatmap (Methode Principale):<br />
       Nous exécutons l'algorithme sur la totalité du terrain de jeux. Les performances de cette version sont très bonnes pour un temps d'exécution important
   ![Situation plus complexe#1](https://github.com/user-attachments/assets/96277982-0d22-4723-86fa-ceb0582a4dd1)
   ![Situation plus complexe#2](https://github.com/user-attachments/assets/e76c4255-7db0-48c3-9ec6-83cb8576200a)
   L'ajout de l'orientation / vitesse de rotation est très important car, il permet d'adapter les zones de passes en fonction de la différence de performance entre les 2 équipes. Ainsi, ajouter plus de critère objectifs, mesurant cette performance améliore la robustesse de l'algorithme.

   - Version Centrée autour des Teammates (Methode Alternative): 
       Nous exécutons l'algorithme à partir des positions de chaques teammates. Les performances de cette version sont inférieures à la version précédente, pour un temps d'exécution bien inférieur
       (Version en Stand By. Elle sera privilégiée si le temps de calcul de la version Heatmap devient problématique)
   ![Passe directe possible](https://github.com/user-attachments/assets/f467a9da-edd0-44cf-a35b-61bf6399292c)
   ![Passe directe impossible](https://github.com/user-attachments/assets/1a2f3cb8-ac7c-4eb5-8f83-d0f44854acd8)
   ![Situation de jeu plus complexe](https://github.com/user-attachments/assets/ccfeb9c5-9dce-4fdc-b169-57673d263804)

---------------------------------
2. Génération de Trajectoire:
     Nous allons travailler sur une méthode permettant d'optimiser le modèle de déplacement existant. Le but est d'assurer le passage des robots, par les waypoints défini par notre modèle de déplacement existant, à des vitesses données.
     Ainsi, nous allons travailler sur la dynamique de déplacement entre 2 waypoints définis, pour assurer les nouvelles contraintes

   
---------------------------------
3. Simulation 2D:
    Nous avons réaliser un petit exemple de test de trajectoire de balle pour améliorer la physique de notre simulation.
    ![Simulation Rebonds après Tir](https://github.com/user-attachments/assets/fcd63165-6dce-489e-a5ab-9dd2196dc2f5)
    ![Simulation Ballon roule au sol après Tir](https://github.com/user-attachments/assets/aa17821d-5599-4e99-94dd-4db8280bca06)


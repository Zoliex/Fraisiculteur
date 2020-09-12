<p align="center"> 
    <img src="https://zupimages.net/up/20/37/b2cv.png"
        height="130">
</p>
<p align="center" >
        <img src="https://img.shields.io/badge/Version%20stable-1.0.1-brightgreen" />
        <img src="https://img.shields.io/badge/Version-1.0.1-orange"/>
        <img src="https://img.shields.io/badge/Made%20with-Visual%20Studio%20Code-blue"/>
        <img src="https://img.shields.io/badge/langage-C%2B%2B-blueviolet"/>
        <img src="https://img.shields.io/badge/Licence-CC%20BY--NC--SA%203.0%20FR-yellow"/>
        <font color="red"><h1>Ce projet et encore en travail</h1></font>
</p>

# Fraisiculteur

Bonjour à tous,
je suppose que vous êtes tous là pour fabriquer un super arrosage automatisé à base d'esp32 (et 8266) ? Si oui alors je vais vous montrer comment en fabriquer un !

Pour commancer téléchargez ce git et décompressez-le

## Pré-Requis

Voici une liste des composants qu'il vous faudra pour fabriquer ce projet (prix: ~ 35€) :

(/!\ lien non aphilié): liste Aliexpress [ICI](https://my.aliexpress.com/wishlist/wish_list_product_list.htm?spm=a2g0s.8937460.0.0.100d2e0ejaGL5q&currentGroupId=1000000000355863)
- Esp32 Devkit v1
- Esp8266
- Pompe
- Module relais
- Des fils
- Un capteur DHT22
- 2 Boutons autobloquants 12mm de Ø
- Un connecteur dc mâle/femelle 12mm de Ø
- Un cable USB
- Un écran LCD I2C 20*4 arduino

_**(Optionnel)**_
- Un panneau solaire 24v
- Un controlleur solaire 12-24v
- Une batterie

# I- Installer **Platformio IDE**

1. Installez Le logiciel **Visual Studio Code** en cliquant [ici](https://code.visualstudio.com/)
2. Une fois **Visual Studio Code** installé ouvrez le et cliquez ici :

![extensions visual studio](https://zupimages.net/up/20/37/rwwz.png)

3. Recherchez **"PlatformIO IDE"** dans la barre de recherche, cliquez sur **"Platformio IDE"** et cliquez sur le bouton install si votre Visual Studio est en Anglais. Une fois l'extension installée redémarrez Visual Studio Code et attendez, vous devriez avoir cette page qui va s'ouvrir :

![platformio home](https://zupimages.net/up/20/37/kktl.png)

Si cela fonctionne passez à l'étape suivante 

# II- Téléverser

Voici quelques instructions pour installer le programme sur le controlleur Arduino.

1. Installez Le logiciel **Visual Studio Code** en cliquant [ici](https://code.visualstudio.com/)
2. Une fois **Visual Studio Code** installé ouvrez le et cliquez ici :

![extensions visual studio](https://zupimages.net/up/20/37/rwwz.png)

3. Recherchez **"PlatformIO IDE"** dans la barre de recherche, cliquez sur **"Platformio IDE"** et cliquez sur le bouton install si votre Visual Studio est en Anglais. Une fois l'extension installée redémarrez Visual Studio Code et attendez, vous devriez avoir cette page qui va s'ouvrir :

![platformio home](https://zupimages.net/up/20/37/kktl.png)

4. cliquez sur **Open Project**, sélectionnez le code source joint et ouvrez-le

5. Branchez votre carte Esp32 et téléversez le en cliquant ici :

![barre upload](https://zupimages.net/up/20/37/7tx1.png)

attendez la fin de l'upload et débranchez votre carte !

### Et voilà !

## Branchements

Pour continuer à fabriquer ce projet il faut brancher l'esp 32 aux différents modules et capteurs :

![schémas](https://zupimages.net/up/20/37/jcab.png)

## Fabriqué avec
- [Visual Studio Code](https://code.visualstudio.com/)
- C++
- [Platformio IDE](https://platformio.org/platformio-ide)

## Versions

- Dernière version stable : v1.0.1

## Auteurs
- **Enzo Coquuelle** _alias_ [Zoliex35](https://github.com/Zoliex35/)

## Licence
Ce projet est sous la licence Attribution - Pas d’Utilisation Commerciale - Partage dans les Mêmes Conditions 3.0 France (CC BY-NC-SA 3.0 FR) - allez voir [ici](https://creativecommons.org/licenses/by-nc-sa/3.0/fr/) pour plus d'infos

<p align="center">
<img src="https://licensebuttons.net/l/by-nc-sa/3.0/fr/88x31.png"/>

</p>
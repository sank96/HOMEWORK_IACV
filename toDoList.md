# Assignement

**A. Scene information:**   
The observed scene contains a car. The car chassy is symmetric about a symmetry plane. The symmetry plane is vertical; several pairs of symmetric point features can be identified, such as vertexes of the rear lights or vertexes of the license plate etc.
Two wheels can also be observed: in each of the two wheels, the circular border between the tire and the rim can easily be identified on the image; the circular borders of the two wheels have the same diameter, and they lie on a common plane, at a same height above the street level. The plane containing the two circular borders is parallel to the symmetry plane of the car chassy (thus it is also vertical).

**B. Image information:**   
An image of a car is taken by a zero-skew camera. Natural camera can not be assumed. Use either Image1 or Image2   

Assignment: write and test a Matlab program that analyzes either Image1 or Image2 to extract the information described below

1. - [ ] **Image feature extraction and selection:**    
Use the learned techniques to find corner features and ellipses in the image. Then manually select those features and those parts of the detected ellipses, that are useful for the subsequent steps.
2. - [ ] **Geometry:**
    1. - [ ] Using constraints on the wheel circles mentioned on point A, determine the ratio between the diameter of the wheel circles and the wheel-to-wheel distance.
    2. - [ ] Using also some of the detected pairs of symmetric features, calibrate the camera by determining the calibration matrix K. Assume the camera is zero-skew (but not natural).
    3. - [ ] Fix a reference frame at a suitable position on the symmetry plane of the car, and reconstruct the 3D position of some of the symmetric pairs of features relative to the above reference.
    4. - [ ] Localize the camera relative to the car reference.


Hint: use normalized image coordinates to reduce numerical errors (e.g., set image size to 1)



# Work in progress
1. **Image feature extraction and selection:**  
    1. caricato le immagini -> messe in un array (non più lavoro su singola immagine ma si può sempre fare)
    2. trasformate in scala di grigio
        - sia con funzione data da matlab e filtro RGB->BW
    3.  troviamo gli  edges
        1. costruiamo i filtri [teoria](theory/2018_Digital_Image_Filters.pdf) (slide 101)
            - Previtt
            - Sobel
        2. applichiamo le derivate
    4. applichiamo le soglie di threshold:
        1. binary threshold
        2. hard threshold
    5. troviamo la conica a partire da 5 punti
        - [ ] utilizzare script inviato da ale su slack (va prima pulita)
            - dilation & erosion

2. **Geometry:**
    1. determine the ratio between the diameter of the wheel circles and the wheel-to-wheel distance
        1.


## TO DO
1. - [ ] (1.3) pulire l'immagine per migliorare la visualizzazione dei corners
    - [ ] usare **erosion e dilation** sentire note audio di Ale


- [ ] ottimizzare il codice per renderlo più veloce
    - [ ] utilizzare il prodotto fra matrici invece che il doppio ciclo `for`
    - [ ] [operazioni su matrici](https://it.mathworks.com/matlabcentral/answers/302402-how-to-select-some-part-of-a-matrix)
    - [x] ottimizzare showProfile
    - [x] ottimizzare findWheel (dove c'è il tic toc) --> farlo come era fatto prima
    - [ ] ottimizzare fromLinesToProfile
    - [ ] trovare le bitangenti --> il problema sono le inverse


## DOING
- [ ] ho implementato un filtro di threshold con soglia, se ne potrebbe fare uno più articolato, prima fare qualche prova per capire quali sono utili!
    - [x] Binary
    - [x] hard

## DONE
- [x] selezionare solo una parte dell'immagine per velocizzare la ricerca dell'immagine in precise posizioni

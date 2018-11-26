1. - [ ] Using constraints on the wheel circles mentioned on point A, determine the ratio between the diameter of the wheel circles and the wheel-to-wheel distance.
---
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
        - utilizzare script inviato da ale su slack (chiedere prima al prof)




# TO DO
- [ ] pulire l'immagine per migliorare la visualizzazione dei corners
    - [ ] usare **erosion e dilation** sentire note audio di Ale


# DOING
- [ ] ho implementato un filtro di threshold con soglia, se ne potrebbe fare uno più articolato, prima fare qualche prova per capire quali sono utili!
    - [x] Binary
    - [x] hard

# DONE
- [x] selezionare solo una parte dell'immagine per velocizzare la ricerca dell'immagine in precise posizioni

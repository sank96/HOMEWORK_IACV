# Generali

### Stampa
```matlab
disp()
```
Permette di stampare a video

# Matrici

### Size

```matlab
[R, C] = size(matrix);
```
### Valori max e min
Calcola i valori massimi e minimi fra tutte le celle
```matlab
MAX = max(max(matrix))
MIN = min(min(matrix))
```
### Operazioni

#### Divisione
`a ./ b` division by dividing each element of _a_ by the corresponding element of _b_. If inputs a and b are not the same size, one of them must be a scalar value.

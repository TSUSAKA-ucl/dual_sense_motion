# PS5 dual sense button layout

* 0--3:	☓○□△
* 4--6: \|/ ??? ≡
* 7--10: stickL, stickR, L1, R1n
* 11--14: up/down/left/right
* 15--16: mouse button / mic 

```
ros2 topic echo --csv /joy |sed -e 's/joy,\([0-9\.\-]\+,\)\{6\}/\t/' -e 's/\(\([01],\)\{4\}\)\(\([01],\)\{3\}\)\(\(\([01],\)\{4\}\)\{2\}\)/\1 \3 \5 /'
```

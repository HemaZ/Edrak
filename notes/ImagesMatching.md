# Images matching

## Key Assumptions

* Images differ only by:
    1. Translation
    2. Brightness
    3. Contrast

## Typical measure of similarity

1. Sum of squared differences (SSD)
$$
SSD = \sum_{m}{(g_2(m)-g_1(m))^2}
$$

2. Sum of absolute differences (SAD)
$$
SAD = \sum_{m}{|g_2(m)-g_1(m)|}
$$

Theses two methods have no invariance against changes in brightness and contrast.

## Cross Correlation (CC)

Best estimate of the offset $[u_b,v_b]$ is given by maximizing the cross correlation coefficient over all possible locations.

g2 is the template image.
g1 is the image we are matching with. 

![`cc`](imgs/cc.png)
![`cc`](imgs/cc2.png)
![`cc`](imgs/cc3.png)
![`cc`](imgs/cc4.png)
![`cc`](imgs/cc5.png)
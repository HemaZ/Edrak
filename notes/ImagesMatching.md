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

$$
[u_b,v_b] = argmax_{u,v} \rho12(u,v)
$$

$$
\rho12(u,v) = \frac{\sigma_{g_1,g_2}(u,v)}{\sigma_{g_1}(u,v)\sigma{g_2}}
$$
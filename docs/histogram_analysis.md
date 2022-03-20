# Normalization
The linear normalization of a grayscale digital image:

<!-- 
<img src="https://latex.codecogs.com/svg.latex?I_{N}=( {\text{newMax}}-{\text{newMin}}   ){\frac  {   I-{\text{Min}}  }{{\text{Max}}-{\text{Min}}}}+{\text{newMin}}" />
-->

<img src="https://latex.codecogs.com/svg.image?I_{N}=(&space;{\text{newMax}}-{\text{newMin}}&space;&space;&space;){\frac&space;&space;{&space;&space;&space;I-{\text{Min}}&space;&space;}{{\text{Max}}-{\text{Min}}}}&plus;{\text{newMin}}" title="https://latex.codecogs.com/svg.image?I_{N}=( {\text{newMax}}-{\text{newMin}} ){\frac { I-{\text{Min}} }{{\text{Max}}-{\text{Min}}}}+{\text{newMin}}" />


# Contrast Stretching
Contrast Stretching =HistogramNormalization= Histogram Stretching

Refs [1](https://homepages.inf.ed.ac.uk/rbf/HIPR2/stretch.htm)
# Histogram Matching: 
Creating new image which has new distribution function (pdf)


# Histogram Equalization:
Creating new image which has new uniform distribution

Refs: [1](https://automaticaddison.com/difference-between-histogram-equalization-and-histogram-matching/)

## Histogram Equalization Vs Histogram Normalization
Numbers are scaled such that either the maximum
count / bin or cumulative bin count equals a specific number
(usually 1). Keeps the shape of the histogram, but modifies
the numbers / bin.

Equalization: The bins are re-distributed such that the
histogram becomes flatter. Changes the shape of the histogram.


# Gamma Correction

the formula for normalization sometime is written as:
new_x=a+ (b-a)*[(x-A)/(B-A)]^Gamma

becasue always 
0< (x-A)/(B-A) < 1 
so if Gamma is biger than one then the result became smaller and if Gamma if smaller than 1 it becames beigger

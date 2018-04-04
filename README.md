# novel

*Novelty detection*, *anomaly detection* and *one-class classification*, all mean the same thing:
Classify samples as positive or negative having only positive training examples.  

The SVM-based framework used by *novel* enables the use of non-linear kernels that help capturing the shape of the
training examples.

*novel* implements the one-class SVM classifier, based on the paper  
[Support Vector Data Description](https://link.springer.com/article/10.1023/B:MACH.0000008084.60811.49)
of Tax et al.  

## Examples
moon-shaped dataset  
Rational Quadratic kernel  

<img src="./tests/images/Figure_1.png" width="50%"/>

## Dependencies
* numpy
* cvxopt

## Installation
~$ pip install novel
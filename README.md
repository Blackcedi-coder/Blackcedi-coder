/DivideAndConquerGreedyApp
    ├── /algorithms
    │   ├── DivideAndConquer.java
    │   ├── Greedy.java
    │   ├── QuickSort.java
    │   ├── MergeSort.java
    │   ├── ClosestPair.java
    │   ├── StrassenMatrixMultiplication.java
    │   ├── Quickhull.java
    │   ├── PrimMST.java
    │   ├── TSPApproximation.java
    │   ├── KruskalMST.java
    │   ├── DijkstraShortestPath.java
    │   ├── HuffmanCodes.java
    ├── /utils
    │   ├── Timer.java
    ├── Main.java
    └── ReadMe.txt
package algorithms;

public interface Algorithm {
    void execute();
    long getExecutionTime();
}
package algorithms;

import java.util.Arrays;

public class QuickSort implements Algorithm {
    private int[] array;
    private long executionTime;

    public QuickSort(int[] array) {
        this.array = array;
    }

    @Override
    public void execute() {
        long startTime = System.nanoTime();
        quickSort(array, 0, array.length - 1);
        executionTime = System.nanoTime() - startTime;
    }

    private void quickSort(int[] array, int low, int high) {
        if (low < high) {
            int pi = partition(array, low, high);
            quickSort(array, low, pi - 1);
            quickSort(array, pi + 1, high);
        }
    }

    private int partition(int[] array, int low, int high) {
        int pivot = array[high];
        int i = low - 1;
        for (int j = low; j < high; j++) {
            if (array[j] <= pivot) {
                i++;
                int temp = array[i];
                array[i] = array[j];
                array[j] = temp;
            }
        }
        int temp = array[i + 1];
        array[i + 1] = array[high];
        array[high] = temp;
        return i + 1;
    }

    @Override
    public long getExecutionTime() {
        return executionTime;
    }

    public int[] getSortedArray() {
        return array;
    }
}
package algorithms;

import java.util.Arrays;

public class MergeSort implements Algorithm {
    private int[] array;
    private long executionTime;

    public MergeSort(int[] array) {
        this.array = array;
    }

    @Override
    public void execute() {
        long startTime = System.nanoTime();
        mergeSort(array, 0, array.length - 1);
        executionTime = System.nanoTime() - startTime;
    }

    private void mergeSort(int[] array, int left, int right) {
        if (left < right) {
            int middle = (left + right) / 2;
            mergeSort(array, left, middle);
            mergeSort(array, middle + 1, right);
            merge(array, left, middle, right);
        }
    }

    private void merge(int[] array, int left, int middle, int right) {
        int n1 = middle - left + 1;
        int n2 = right - middle;
        int[] leftArray = new int[n1];
        int[] rightArray = new int[n2];
        System.arraycopy(array, left, leftArray, 0, n1);
        System.arraycopy(array, middle + 1, rightArray, 0, n2);
        int i = 0, j = 0;
        int k = left;
        while (i < n1 && j < n2) {
            if (leftArray[i] <= rightArray[j]) {
                array[k++] = leftArray[i++];
            } else {
                array[k++] = rightArray[j++];
            }
        }
        while (i < n1) {
            array[k++] = leftArray[i++];
        }
        while (j < n2) {
            array[k++] = rightArray[j++];
        }
    }

    @Override
    public long getExecutionTime() {
        return executionTime;
    }

    public int[] getSortedArray() {
        return array;
    }
}
package algorithms;

import java.awt.Point;
import java.util.Arrays;

public class ClosestPair implements Algorithm {
    private Point[] points;
    private double minDistance;
    private long executionTime;

    public ClosestPair(Point[] points) {
        this.points = points;
    }

    @Override
    public void execute() {
        long startTime = System.nanoTime();
        minDistance = closestPair(points);
        executionTime = System.nanoTime() - startTime;
    }

    private double closestPair(Point[] points) {
        Arrays.sort(points, (p1, p2) -> Double.compare(p1.getX(), p2.getX()));
        return closestPairRecursive(points, 0, points.length - 1);
    }

    private double closestPairRecursive(Point[] points, int left, int right) {
        if (right - left <= 3) {
            return bruteForce(points, left, right);
        }
        int mid = (left + right) / 2;
        double leftDist = closestPairRecursive(points, left, mid);
        double rightDist = closestPairRecursive(points, mid + 1, right);
        double minDist = Math.min(leftDist, rightDist);

        return Math.min(minDist, stripClosest(points, left, right, (points[mid].x + points[mid + 1].x) / 2, minDist));
    }

    private double bruteForce(Point[] points, int left, int right) {
        double minDist = Double.MAX_VALUE;
        for (int i = left; i <= right; i++) {
            for (int j = i + 1; j <= right; j++) {
                double dist = points[i].distance(points[j]);
                if (dist < minDist) {
                    minDist = dist;
                }
            }
        }
        return minDist;
    }

    private double stripClosest(Point[] points, int left, int right, double midX, double minDist) {
        Point[] strip = Arrays.stream(points)
            .filter(p -> Math.abs(p.getX() - midX) < minDist)
            .toArray(Point[]::new);
        Arrays.sort(strip, (p1, p2) -> Double.compare(p1.getY(), p2.getY()));

        double min = minDist;
        for (int i = 0; i < strip.length; i++) {
            for (int j = i + 1; j < strip.length && strip[j].getY() - strip[i].getY() < min; j++) {
                double dist = strip[i].distance(strip[j]);
                if (dist < min) {
                    min = dist;
                }
            }
        }
        return min;
    }

    @Override
    public long getExecutionTime() {
        return executionTime;
    }

    public double getMinDistance() {
        return minDistance;
    }
}
package algorithms;

public class StrassenMatrixMultiplication implements Algorithm {
    private int[][] matrixA;
    private int[][] matrixB;
    private int[][] result;
    private long executionTime;

    public StrassenMatrixMultiplication(int[][] matrixA, int[][] matrixB) {
        this.matrixA = matrixA;
        this.matrixB = matrixB;
        this.result = new int[matrixA.length][matrixB[0].length];
    }

    @Override
    public void execute() {
        long startTime = System.nanoTime();
        result = strassen(matrixA, matrixB);
        executionTime = System.nanoTime() - startTime;
    }

    private int[][] strassen(int[][] A, int[][] B) {
        int n = A.length;
        if (n == 1) {
            int[][] C = new int[1][1];
            C[0][0] = A[0][0] * B[0][0];
            return C;
        }
        int newSize = n / 2;
        int[][] A11 = new int[newSize][newSize];
        int[][] A12 = new int[newSize][newSize];
        int[][] A21 = new int[newSize][newSize];
        int[][] A22 = new int[newSize][newSize];
        int[][] B11 = new int[newSize][newSize];
        int[][] B12 = new int[newSize][newSize];
        int[][] B21 = new int[newSize][newSize];
        int[][] B22 = new int[newSize][newSize];

        split(A, A11, A12, A21, A22);
        split(B, B11, B12, B21, B22);

        int[][] M1 = strassen(add(A11,

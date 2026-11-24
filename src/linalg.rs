//! Small dense linear algebra: 3 x 3 matrices for the filters and a dense
//! symmetric solver for the factor graph.

/// 3 x 3 matrix, row major.
pub type Mat3 = [[f64; 3]; 3];

/// 3 x 3 identity.
pub const IDENTITY3: Mat3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

/// `a * b`.
pub fn mul3(a: &Mat3, b: &Mat3) -> Mat3 {
    let mut c = [[0.0; 3]; 3];
    for (i, row) in c.iter_mut().enumerate() {
        for (j, v) in row.iter_mut().enumerate() {
            *v = (0..3).map(|k| a[i][k] * b[k][j]).sum();
        }
    }
    c
}

/// Transpose.
pub fn transpose3(a: &Mat3) -> Mat3 {
    let mut t = [[0.0; 3]; 3];
    for (i, row) in a.iter().enumerate() {
        for (j, v) in row.iter().enumerate() {
            t[j][i] = *v;
        }
    }
    t
}

/// Inverse, or `None` when singular.
pub fn inverse3(m: &Mat3) -> Option<Mat3> {
    let c = |r0: usize, r1: usize, c0: usize, c1: usize| m[r0][c0] * m[r1][c1] - m[r0][c1] * m[r1][c0];
    let cof = [
        [c(1, 2, 1, 2), -c(1, 2, 0, 2), c(1, 2, 0, 1)],
        [-c(0, 2, 1, 2), c(0, 2, 0, 2), -c(0, 2, 0, 1)],
        [c(0, 1, 1, 2), -c(0, 1, 0, 2), c(0, 1, 0, 1)],
    ];
    let det = m[0][0] * cof[0][0] + m[0][1] * cof[0][1] + m[0][2] * cof[0][2];
    if det.abs() < 1e-300 || !det.is_finite() {
        return None;
    }
    let mut inv = [[0.0; 3]; 3];
    for (i, row) in inv.iter_mut().enumerate() {
        for (j, v) in row.iter_mut().enumerate() {
            *v = cof[j][i] / det;
        }
    }
    Some(inv)
}

/// Inverse of a symmetric 2 x 2 matrix, or `None` when singular.
pub fn inverse2(m: [[f64; 2]; 2]) -> Option<[[f64; 2]; 2]> {
    let det = m[0][0] * m[1][1] - m[0][1] * m[1][0];
    if det.abs() < 1e-300 || !det.is_finite() {
        return None;
    }
    Some([[m[1][1] / det, -m[0][1] / det], [-m[1][0] / det, m[0][0] / det]])
}

/// Solves `a x = b` for symmetric positive definite `a` (n x n, row major)
/// by Cholesky factorization, in place: `b` becomes `x`. Returns false when
/// `a` is not positive definite.
pub fn cholesky_solve(a: &mut [f64], b: &mut [f64], n: usize) -> bool {
    for j in 0..n {
        let mut d = a[j * n + j];
        for k in 0..j {
            d -= a[j * n + k] * a[j * n + k];
        }
        if d <= 0.0 || !d.is_finite() {
            return false;
        }
        let d = d.sqrt();
        a[j * n + j] = d;
        for i in j + 1..n {
            let mut s = a[i * n + j];
            for k in 0..j {
                s -= a[i * n + k] * a[j * n + k];
            }
            a[i * n + j] = s / d;
        }
    }
    for i in 0..n {
        let mut s = b[i];
        for k in 0..i {
            s -= a[i * n + k] * b[k];
        }
        b[i] = s / a[i * n + i];
    }
    for i in (0..n).rev() {
        let mut s = b[i];
        for k in i + 1..n {
            s -= a[k * n + i] * b[k];
        }
        b[i] = s / a[i * n + i];
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn inverse3_inverts() {
        let m = [[4.0, 1.0, 0.5], [1.0, 3.0, 0.2], [0.5, 0.2, 2.0]];
        let p = mul3(&m, &inverse3(&m).unwrap());
        for (i, row) in p.iter().enumerate() {
            for (j, v) in row.iter().enumerate() {
                assert!((v - IDENTITY3[i][j]).abs() < 1e-12);
            }
        }
    }

    #[test]
    fn cholesky_solves_spd_systems() {
        let mut a = vec![4.0, 1.0, 0.5, 1.0, 3.0, 0.2, 0.5, 0.2, 2.0];
        let x = [1.0, -2.0, 0.5];
        let mut b: Vec<f64> = (0..3).map(|i| (0..3).map(|k| a[i * 3 + k] * x[k]).sum()).collect();
        assert!(cholesky_solve(&mut a, &mut b, 3));
        for (bi, xi) in b.iter().zip(x) {
            assert!((bi - xi).abs() < 1e-12);
        }
        let mut not_pd = vec![1.0, 2.0, 2.0, 1.0];
        assert!(!cholesky_solve(&mut not_pd, &mut [1.0, 1.0], 2));
    }
}

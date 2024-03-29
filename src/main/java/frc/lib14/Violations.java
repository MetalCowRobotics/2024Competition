package frc.lib14;

public class Violations {
    private final int rightEncoder = 0;

    public void doCheckStyle() { // 1
        double baseSpeed = 0;
        int a = 2, b = 3, a1 = 0, b1 = 0, a2 = 0, b2 = 0, c = 0, d = 1, c1 = 1;
        if (a != b) { // 2
            doIFs(a1, b1, a2, b2, c1);
        } else if (c != d) { // 7
            a += 1;
        }
        if (b > a || (baseSpeed == rightEncoder)) {
            doNothing();
        }
    }

    private void doIFs(int a1, int b1, int a2, int b2, int c1) {
        int d1 = 1;
        if (a1 != b1 // 3
                && c1 != d1) { // 4
            doNothing();
        } else if (a2 != b2 // 5
                || c1 < d1) { // 6
            doNothing();
        } else {
            doNothing();
        }
    }

    private void doNothing() {}
}
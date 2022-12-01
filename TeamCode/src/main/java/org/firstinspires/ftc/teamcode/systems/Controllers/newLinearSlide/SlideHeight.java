package org.firstinspires.ftc.teamcode.systems.Controllers.newLinearSlide;


public enum SlideHeight {
    Floor,
    Low,
    Medium,
    High,
    Ground;

    private static final int[] Heights;

    static {
        SlideHeight[] values = SlideHeight.values();
        Heights = new int[values.length];
        for (int i = 0; i < values.length; i++) {
            Heights[i] = SlideHeight.getEncoderCountFromEnum(values[i]);
        }
    }

    public static int getEncoderCountFromEnum(SlideHeight level) {
        switch (level) {
            case High:
                return 584;
            case Medium:
                return 233;
            case Low:
            case Floor:
            case Ground:
            default:
                return 0;
        }
    }

    public static SlideHeight getClosestLevel(int currentLevel) {
        SlideHeight[] levels = SlideHeight.values();

        int[] distances = new int[levels.length];

        for (int i = 0; i < levels.length; i++) {
            distances[i] = Math.abs(Heights[i] - currentLevel);
        }

        double smallestDistance = Double.MAX_VALUE;
        int indexOfSmallestValue = -1;

        for (int i = 0; i < levels.length; i++) {
            if (distances[i] < smallestDistance) {
                smallestDistance = distances[i];

                indexOfSmallestValue = i;
            }
        }

        return levels[indexOfSmallestValue];
    }



    private static int getLevelIndex(SlideHeight level) {
        switch (level) {
            case Floor:
            case Ground:
                return 0;
            case Low:
                return 1;
            case Medium:
                return 2;
            case High:
                return 3;
        }
        return 0;
    }

    private static SlideHeight getLevelFromIndex(int value) {
        switch (value) {
            case 0:
                return Floor;
            case 1:
                return Low;
            case 2:
                return Medium;
            case 3:
                return High;
        }

        if (value > 3) {
            return High;
        }
        return Floor;
    }

    public SlideHeight add(int amount) {
        return SlideHeight.getLevelFromIndex(getLevelIndex(this) + amount);
    }

    public SlideHeight subtract(int amount) {
        return SlideHeight.getLevelFromIndex(getLevelIndex(this) - amount);
    }


}
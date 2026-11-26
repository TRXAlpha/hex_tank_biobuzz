package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.ArrayList;
import java.util.List;

/*
 * HIVE CELL center from its 4-tag AprilTag cluster (game manual 9.9). Same math in teleop and calibration.
 *   1. Each tag -> point on the floor, from the camera (range, tx, ty, camera pitch).
 *   2. Row direction from the two outermost tags seen (else: assume the row faces the camera).
 *   3. Center = average of (tag point - tag offset along the row). Works with 1 to 4 tags.
 */
@Config
public class CellVision {
    public static double CAM_FWD_MM = 30;                               // lens ahead of the turret pivot
    public static double TAG_OUTER_MM = 165.1, TAG_INNER_MM = 69.85;    // tag centers from the cluster center
    public static double MIN_RANGE_MM = 200, MAX_RANGE_MM = 6000;       // outside = bad detection

    /** One tag: cluster base id, index 0..3 in the cluster, raw Limelight range (mm), tx, ty (deg). */
    public static class Tag {
        public final int base, i;
        public final double range, tx, ty;
        public Tag(int base, int i, double range, double tx, double ty) {
            this.base = base; this.i = i; this.range = range; this.tx = tx; this.ty = ty;
        }
    }

    /** Tags of the cluster (one of the given base ids) with the most tags in this frame. Empty if none. */
    public static List<Tag> tags(LLResult r, int... bases) {
        List<Tag> best = new ArrayList<>();
        for (int base : bases) {
            List<Tag> now = new ArrayList<>();
            boolean[] has = new boolean[4];
            for (LLResultTypes.FiducialResult f : r.getFiducialResults()) {
                int i = f.getFiducialId() - base;
                if (i < 0 || i > 3 || has[i]) continue;
                Position p = f.getTargetPoseCameraSpace().getPosition().toUnit(DistanceUnit.MM);
                double range = Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
                if (range < MIN_RANGE_MM || range > MAX_RANGE_MM) continue;
                has[i] = true;
                now.add(new Tag(base, i, range, f.getTargetXDegrees(), f.getTargetYDegrees()));
            }
            if (now.size() > best.size()) best = now;
        }
        return best;
    }

    /** CELL center from the TURRET PIVOT, {forward, left} in mm. Null if no tags. */
    public static double[] center(List<Tag> tags, double pitchDeg, double scale) {
        if (tags.isEmpty()) return null;
        double[] off = {-TAG_OUTER_MM, -TAG_INNER_MM, TAG_INNER_MM, TAG_OUTER_MM};
        double[] px = new double[4], py = new double[4];
        boolean[] has = new boolean[4];
        for (Tag t : tags) {
            double horiz = t.range * scale * Math.cos(Math.toRadians(pitchDeg + t.ty));
            double b = -Math.toRadians(t.tx);                  // tx is + to the right -> + = left
            px[t.i] = horiz * Math.cos(b);                     // forward from the lens
            py[t.i] = horiz * Math.sin(b);                     // left of the lens
            has[t.i] = true;
        }
        int lo = -1, hi = -1;
        for (int i = 0; i < 4; i++) if (has[i]) { if (lo < 0) lo = i; hi = i; }
        double ux = 0, uy = -1;                                // 1 tag: row runs to the camera's right
        if (hi > lo) {
            ux = px[hi] - px[lo];
            uy = py[hi] - py[lo];
            double len = Math.hypot(ux, uy);
            ux /= len;
            uy /= len;
        }
        double cx = 0, cy = 0;
        for (int i = 0; i < 4; i++) {
            if (!has[i]) continue;
            cx += px[i] - off[i] * ux;
            cy += py[i] - off[i] * uy;
        }
        int n = tags.size();
        return new double[]{cx / n + CAM_FWD_MM, cy / n};
    }

    /** Distance turret pivot -> CELL center in cm, or -1. */
    public static double distCm(List<Tag> tags, double pitchDeg, double scale) {
        double[] c = center(tags, pitchDeg, scale);
        return c == null ? -1 : Math.hypot(c[0], c[1]) / 10.0;
    }
}

package top.kzre.krro.curve.bezier2d;

import top.kzre.curve.bezier2d.Curve;

import java.util.ArrayList;

import java.util.concurrent.ConcurrentLinkedQueue;

/**
 * Curve 对象池，用于复用 Curve 实例，减少 GC 压力。
 * 所有方法均为静态方法，池本身是线程安全的。
 */
public final class CurvePool {

    private static final ConcurrentLinkedQueue<Curve> pool = new ConcurrentLinkedQueue<>();

    private CurvePool() {}

    /**
     * 从池中借出一个空的 Curve 对象。
     * 如果池为空，则新建一个 Curve（内部点列表为 ArrayList，closed=false）。
     */
    public static Curve borrowCurve() {
        Curve c = pool.poll();
        if (c == null) {
            c = new Curve(new ArrayList<>(), false);
        }
        // 确保借出时是干净的（但理论上归还时已清空）
        return c;
    }

    /**
     * 将 Curve 对象归还池中。归还前会清空其控制点列表并重置 closed 状态。
     */
    public static void returnCurve(Curve c) {
        if (c != null) {
            // 清空内部状态以便下次复用
            c.getPoints().clear();
            c.setClosed(false);
            pool.offer(c);
        }
    }

    /**
     * 获取当前池中对象数量（仅用于调试）。
     */
    public static int size() {
        return pool.size();
    }
}
(ns top.kzre.krro.curve.bezier2d.core
  "Bézier2D 的纯 Clojure 封装。所有函数接收/返回 EDN 数据，
   内部通过 CurvePool 借用 Java Curve 对象执行计算，用完即还。"
  (:refer-clojure :exclude [reverse])
  (:import (java.util ArrayList Collection)
           (top.kzre.curve.bezier2d AABB ArcLengthTable Bezier2D ClosestPointResult ControlPoint Curve Pair)
           (top.kzre.krro.curve.bezier2d CurvePool)))

;; ═══════════════════════════════════════════════════════
;; EDN ↔ Java 转换
;; ═══════════════════════════════════════════════════════

(defn curve->edn
  "将 Java Curve 转换为 EDN map。"
  [^Curve c]
  {:closed (.isClosed c)
   :points (mapv (fn [^ControlPoint p]
                   {:x (.getX p) :y (.getY p)
                    :dx1 (.getDx1 p) :dy1 (.getDy1 p)
                    :dx2 (.getDx2 p) :dy2 (.getDy2 p)
                    :g1 (.isG1 p)})
                 (.getPoints c))})

(defn edn->curve!
  "将 EDN map 的数据设置到给定的 Curve 对象中（修改传入的 Curve）。"
  [^Curve c m]
  (let [pts (mapv (fn [p]
                    (ControlPoint.
                      (double (:x p)) (double (:y p))
                      (double (:dx1 p)) (double (:dy1 p))
                      (double (:dx2 p)) (double (:dy2 p))
                      (boolean (:g1 p))))
                  (:points m))]
    (.setPoints c (ArrayList. ^Collection pts))
    (.setClosed c (boolean (:closed m)))
    c))

(defn pair->edn [^Pair p]
  {:x (.getX p) :y (.getY p)})

(defn edn->pair [m]
  (Pair. (:x m) (:y m)))

(defn aabb->edn [^AABB aabb]
  {:min-x (.getMinX aabb) :min-y (.getMinY aabb)
   :max-x (.getMaxX aabb) :max-y (.getMaxY aabb)})

(defn closest-result->edn [^ClosestPointResult r]
  {:point (pair->edn (.getPoint r))
   :t (.getT r)
   :distance (.getDistance r)})

;; ═══════════════════════════════════════════════════════
;; 核心 API
;; ═══════════════════════════════════════════════════════

(defn evaluate
  "在参数 t 处求值，返回点 {:x :y}。"
  [curve-edn t]
  (let [c (CurvePool/borrowCurve)]
    (try
      (edn->curve! c curve-edn)
      (pair->edn (Bezier2D/eval c (double t)))
      (finally
        (CurvePool/returnCurve c)))))

(defn deriv
  "一阶导数，返回点 {:x :y}。"
  [curve-edn t]
  (let [c (CurvePool/borrowCurve)]
    (try
      (edn->curve! c curve-edn)
      (pair->edn (Bezier2D/deriv c (double t)))
      (finally
        (CurvePool/returnCurve c)))))

(defn deriv2
  "二阶导数，返回点 {:x :y}。"
  [curve-edn t]
  (let [c (CurvePool/borrowCurve)]
    (try
      (edn->curve! c curve-edn)
      (pair->edn (Bezier2D/deriv2 c (double t)))
      (finally
        (CurvePool/returnCurve c)))))

(defn unit-tangent
  "单位切向量。"
  [curve-edn t]
  (let [c (CurvePool/borrowCurve)]
    (try
      (edn->curve! c curve-edn)
      (pair->edn (Bezier2D/unitTangent c (double t)))
      (finally
        (CurvePool/returnCurve c)))))

(defn unit-normal
  "单位法向量（左手系）。"
  [curve-edn t]
  (let [c (CurvePool/borrowCurve)]
    (try
      (edn->curve! c curve-edn)
      (pair->edn (Bezier2D/unitNormal c (double t)))
      (finally
        (CurvePool/returnCurve c)))))

(defn curvature
  "曲率。"
  [curve-edn t]
  (let [c (CurvePool/borrowCurve)]
    (try
      (edn->curve! c curve-edn)
      (Bezier2D/curvature c (double t))
      (finally
        (CurvePool/returnCurve c)))))

(defn aabb
  "计算包围盒，返回 {:min-x :min-y :max-x :max-y}。"
  [curve-edn]
  (let [c (CurvePool/borrowCurve)]
    (try
      (edn->curve! c curve-edn)
      (aabb->edn (Bezier2D/aabb c))
      (finally
        (CurvePool/returnCurve c)))))

(defn split
  "在参数 t 处分割曲线，返回 [left-edn right-edn]。"
  [curve-edn t]
  (let [c (CurvePool/borrowCurve)
        left (CurvePool/borrowCurve)
        right (CurvePool/borrowCurve)]
    (try
      (edn->curve! c curve-edn)
      (Bezier2D/split c (double t) left right)
      [(curve->edn left) (curve->edn right)]
      (finally
        (CurvePool/returnCurve c)
        (CurvePool/returnCurve left)
        (CurvePool/returnCurve right)))))

(defn divide
  "在控制点索引 idx 处分割，返回 [left-edn right-edn]。"
  [curve-edn idx]
  (let [c (CurvePool/borrowCurve)
        left (CurvePool/borrowCurve)
        right (CurvePool/borrowCurve)]
    (try
      (edn->curve! c curve-edn)
      (Bezier2D/divide c (int idx) left right)
      [(curve->edn left) (curve->edn right)]
      (finally
        (CurvePool/returnCurve c)
        (CurvePool/returnCurve left)
        (CurvePool/returnCurve right)))))

(defn translate
  "平移曲线，返回新曲线的 EDN。"
  [curve-edn dx dy]
  (let [c (CurvePool/borrowCurve)]
    (try
      (edn->curve! c curve-edn)
      (let [new-curve (Bezier2D/translate c (double dx) (double dy))]
        (try
          (curve->edn new-curve)
          (finally
            (CurvePool/returnCurve new-curve))))
      (finally
        (CurvePool/returnCurve c)))))

(defn scale
  "缩放曲线，返回新曲线的 EDN。"
  [curve-edn sx sy cx cy]
  (let [c (CurvePool/borrowCurve)]
    (try
      (edn->curve! c curve-edn)
      (let [new-curve (Bezier2D/scale c (double sx) (double sy) (double cx) (double cy))]
        (try
          (curve->edn new-curve)
          (finally
            (CurvePool/returnCurve new-curve))))
      (finally
        (CurvePool/returnCurve c)))))

(defn join
  "连接两条曲线，返回新曲线的 EDN。"
  [left-edn right-edn]
  (let [left (CurvePool/borrowCurve)
        right (CurvePool/borrowCurve)]
    (try
      (edn->curve! left left-edn)
      (edn->curve! right right-edn)
      (let [joined (Bezier2D/join left right)]
        (try
          (curve->edn joined)
          (finally
            (CurvePool/returnCurve joined))))
      (finally
        (CurvePool/returnCurve left)
        (CurvePool/returnCurve right)))))

(defn fit
  "从点序列拟合贝塞尔曲线，返回曲线的 EDN。
   xs, ys 为 double-array，max-error 为误差阈值，max-seg 为最大段数。"
  [^doubles xs ^doubles ys max-error max-seg]
  (let [fitted (Bezier2D/fit xs ys (double max-error) (int max-seg))]
    (try
      (curve->edn fitted)
      (finally
        (CurvePool/returnCurve fitted)))))

(defn insert-point
  "在参数 t 处插入控制点，返回修改后的曲线 EDN。"
  [curve-edn t]
  (let [c (CurvePool/borrowCurve)]
    (try
      (edn->curve! c curve-edn)
      (Bezier2D/insertPoint c (double t))
      (curve->edn c)
      (finally
        (CurvePool/returnCurve c)))))

(defn delete-point
  "删除索引 idx 处的控制点，返回修改后的曲线 EDN。"
  [curve-edn idx]
  (let [c (CurvePool/borrowCurve)]
    (try
      (edn->curve! c curve-edn)
      (Bezier2D/deletePoint c (int idx))
      (curve->edn c)
      (finally
        (CurvePool/returnCurve c)))))

(defn reform
  "重设控制点数目为 count，返回新曲线的 EDN。"
  [curve-edn count]
  (let [c (CurvePool/borrowCurve)]
    (try
      (edn->curve! c curve-edn)
      (Bezier2D/reform c (int count))
      (curve->edn c)
      (finally
        (CurvePool/returnCurve c)))))

(defn reverse
  "反向曲线，返回新曲线的 EDN。"
  [curve-edn]
  (let [c (CurvePool/borrowCurve)]
    (try
      (edn->curve! c curve-edn)
      (let [rev (Bezier2D/reverse c)]
        (try
          (curve->edn rev)
          (finally
            (CurvePool/returnCurve rev))))
      (finally
        (CurvePool/returnCurve c)))))

(defn offset
  "偏移曲线（近似），返回新曲线的 EDN。"
  [curve-edn distance]
  (let [c (CurvePool/borrowCurve)]
    (try
      (edn->curve! c curve-edn)
      (let [off (Bezier2D/offset c (double distance))]
        (try
          (curve->edn off)
          (finally
            (CurvePool/returnCurve off))))
      (finally
        (CurvePool/returnCurve c)))))

(defn closest-point
  "求点到曲线的最近点，返回 {:point {:x :y}, :t :distance}。"
  [curve-edn point-edn]
  (let [c (CurvePool/borrowCurve)]
    (try
      (edn->curve! c curve-edn)
      (let [p (edn->pair point-edn)
            result (Bezier2D/closestPoint c p)]
        (closest-result->edn result))
      (finally
        (CurvePool/returnCurve c)))))

(defn sample
  "均匀采样 count 个点（包含首尾），返回点向量。"
  [curve-edn count]
  (let [c (CurvePool/borrowCurve)]
    (try
      (edn->curve! c curve-edn)
      (let [pairs (Bezier2D/sample c (int count))]
        (mapv pair->edn pairs))
      (finally
        (CurvePool/returnCurve c)))))

;; ── 弧长计算 ──────────────────────────────────────
(defn arc-length-table
  "构造弧长表，返回一个可查询的 map，包含 :total 和查询函数。"
  [curve-edn samples]
  (let [c (CurvePool/borrowCurve)]
    (try
      (edn->curve! c curve-edn)
      (let [^ArcLengthTable table (ArcLengthTable. c (int samples))]
        {:total (.totalLength table)
         :get-length (fn [t] (.getLength table (double t)))
         :get-t      (fn [s] (.getT table (double s)))})
      (finally
        (CurvePool/returnCurve c)))))
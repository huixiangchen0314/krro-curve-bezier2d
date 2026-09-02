(ns top.kzre.krro.curve.bezier2d.core
  "Bézier2D 的纯 Clojure 封装。所有函数接收/返回 EDN 数据，
   内部直接构造 Java Curve 对象，无需池化。"
  (:refer-clojure :exclude [reverse])
  (:import (java.util ArrayList Collection)
           (top.kzre.curve.bezier2d AABB ArcLengthTable Bezier2D ClosestPointResult ControlPoint Curve Pair Segments)))

;; ── EDN ↔ Java 转换 ──────────────────────────────────

(defn edn->curve
  "将 EDN map 转换为 Java Curve 对象（新分配）。"
  [m]
  (let [pts (mapv (fn [p]
                    (ControlPoint.
                      (double (:x p)) (double (:y p))
                      (double (:dx1 p)) (double (:dy1 p))
                      (double (:dx2 p)) (double (:dy2 p))
                      (boolean (:g1 p))))
                  (:points m))]
    (Curve. (ArrayList. ^Collection pts) (boolean (:closed m)))))

(defn curve->edn [^Curve c]
  {:closed (.isClosed c)
   :points (mapv (fn [^ControlPoint p]
                   {:x (.getX p) :y (.getY p)
                    :dx1 (.getDx1 p) :dy1 (.getDy1 p)
                    :dx2 (.getDx2 p) :dy2 (.getDy2 p)
                    :g1 (.isG1 p)})
                 (.getPoints c))})
(defn edn->point [p]
  (ControlPoint.
    (double (:x p)) (double (:y p))
    (double (:dx1 p)) (double (:dy1 p))
    (double (:dx2 p)) (double (:dy2 p))
    (boolean (:g1 p))))

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

;; ── 核心 API（直接构造，无池化） ──────────────────

(defn evaluate [curve-edn t]
  (let [c (edn->curve curve-edn)]
    (pair->edn (Bezier2D/eval c (double t)))))

(defn deriv [curve-edn t]
  (let [c (edn->curve curve-edn)]
    (pair->edn (Bezier2D/deriv c (double t)))))

(defn deriv2 [curve-edn t]
  (let [c (edn->curve curve-edn)]
    (pair->edn (Bezier2D/deriv2 c (double t)))))

(defn tangent [curve-edn t]
  (let [c (edn->curve curve-edn)]
    (pair->edn (Bezier2D/unitTangent c (double t)))))

(defn normal [curve-edn t]
  (let [c (edn->curve curve-edn)]
    (pair->edn (Bezier2D/unitNormal c (double t)))))

(defn curvature [curve-edn t]
  (let [c (edn->curve curve-edn)]
    (Bezier2D/curvature c (double t))))

(defn aabb
  "计算曲线的包围盒。
   单参数时返回整条曲线的 AABB。
   多参数时返回这些控制点相邻段的 AABB 并集。"
  ([curve-edn]
   (let [c (edn->curve curve-edn)]
     (aabb->edn (Bezier2D/aabb c))))
  ([curve-edn & idxs]
   (let [c (edn->curve curve-edn)]
     (aabb->edn
       (reduce (fn [acc idx]
                 (let [box (Bezier2D/aabb c (int idx))]
                   (if acc
                     (.merge box acc)
                     box)))
               nil
               idxs)))))

(defn seg-aabb
  [curve-edn & idxs]
  (let [^Curve c (edn->curve curve-edn)]
    (aabb->edn
      (reduce (fn [acc idx]
                (let [seg (.getSegment c idx)
                      box (Segments/aabb seg)]
                  (if acc
                    (.merge box acc)
                    box)))
              nil
              idxs))))

(defn merge-aabb
  "合并多个 AABB map，返回合并后的 AABB map，若所有输入为 nil 则返回 nil。"
  [& aabbs]
  (let [valid (filter some? aabbs)]
    (when (seq valid)
      (reduce (fn [acc aabb]
                {:min-x (min (:min-x acc) (:min-x aabb))
                 :min-y (min (:min-y acc) (:min-y aabb))
                 :max-x (max (:max-x acc) (:max-x aabb))
                 :max-y (max (:max-y acc) (:max-y aabb))})
              (first valid)
              (rest valid)))))


(defn split [curve-edn t]
  (let [c (edn->curve curve-edn)
        left (Curve.)
        right (Curve.)]
    (Bezier2D/split c (double t) left right)
    [(curve->edn left) (curve->edn right)]))

(defn divide [curve-edn idx]
  (let [c (edn->curve curve-edn)
        left (Curve.)
        right (Curve.)]
    (Bezier2D/divide c (int idx) left right)
    [(curve->edn left) (curve->edn right)]))

(defn translate
  "平移曲线。
   单参数形式：整体平移整条曲线。
   多参数形式：平移指定索引的控制点（不影响其他点及其手柄）。"
  ([curve-edn dx dy]
   (let [c (edn->curve curve-edn)]
     (curve->edn (Bezier2D/translate c (double dx) (double dy)))))
  ([curve-edn dx dy & idxs]
   (if (empty? idxs)
     curve-edn
     (let [points (get-in curve-edn [:points])
           idx-set (set idxs)
           new-points (map-indexed (fn [idx p]
                                     (if (idx-set idx)
                                       (assoc p :x (+ (:x p) dx) :y (+ (:y p) dy))
                                       p))
                                   points)]
       (assoc curve-edn :points new-points)))))

(defn scale [curve-edn sx sy cx cy]
  (let [c (edn->curve curve-edn)]
    (curve->edn (Bezier2D/scale c (double sx) (double sy) (double cx) (double cy)))))

(defn join [left-edn right-edn]
  (let [left (edn->curve left-edn)
        right (edn->curve right-edn)]
    (curve->edn (Bezier2D/join left right))))

(defn fit [^doubles xs ^doubles ys max-error max-seg]
  (curve->edn (Bezier2D/fit xs ys (double max-error) (int max-seg))))

(defn insert-point [curve-edn t]
  (let [c (edn->curve curve-edn)]
    (Bezier2D/insertPoint c (double t))
    (curve->edn c)))


(defn delete-point [curve-edn idx]
  (let [c (edn->curve curve-edn)]
    (Bezier2D/deletePoint c (int idx))
    (curve->edn c)))

(defn reform [curve-edn count]
  (let [c (edn->curve curve-edn)]
    (Bezier2D/reform c (int count))
    (curve->edn c)))

(defn reverse [curve-edn]
  (let [c (edn->curve curve-edn)]
    (curve->edn (Bezier2D/reverse c))))

(defn offset [curve-edn distance]
  (let [c (edn->curve curve-edn)]
    (curve->edn (Bezier2D/offset c (double distance)))))

(defn closest-point [curve-edn point-edn]
  (let [c (edn->curve curve-edn)
        p (edn->pair point-edn)]
    (closest-result->edn (Bezier2D/closestPoint c p))))

(defn sample [curve-edn count]
  (let [c (edn->curve curve-edn)]
    (mapv pair->edn (Bezier2D/sample c (int count)))))

(defn arc-length-table [curve-edn samples]
  (let [c (edn->curve curve-edn)
        table (ArcLengthTable. c (int samples))]
    {:total (.totalLength table)
     :get-length (fn [t] (.getLength table (double t)))
     :get-t      (fn [s] (.getT table (double s)))}))
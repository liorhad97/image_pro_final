from scanner import Scanner
from test_angle_motors import move_by_angle_and_distance
from main import build_arg_parser, build_config

def integrated_system():
    print("[INFO] התחלת המערכת המשולבת.")

    # הגדרת סורק
    parser = build_arg_parser()
    args = parser.parse_args()
    config = build_config(args)
    scanner = Scanner(config)

    # הפעלת סריקה
    print("[INFO] הפעלת סריקה לזיהוי אובייקטים.")
    result = scanner.run()

    if result is not None:
        #print("[INFO] אובייקט זוהה. תוצאה:", result)
        print( getattr(result, "angle_deg", 90))
        print( getattr(result, "distance_cm", 100)) # הדפסת התוצאה המלאה לצורך בדיקה
        # שימוש בתוצאה כדי להזיז את הרובוט
        angle = getattr(result, "angle_deg", 90)  # זווית ברירת מחדל 90 מעלות
        distance_cm = getattr(result, "distance_cm", 100)  # מרחק ברירת מחדל 100 ס"מ
        
        print(f"[INFO] מזיז את הרובוט לזווית {angle} ולמרחק {distance_cm} ס\"מ.")
        if distance_cm is not None and distance_cm > 2.5:  # בדיקה שהמרחק חיובי לפני התנועה
            move_by_angle_and_distance(angle, distance_cm-2.5)  # תנועה למרחק פחות 2.5 ס"מ כדי לא להתקרב יותר מדי
    else:
        print("[INFO] לא זוהה אובייקט. המערכת נעצרת.")

if __name__ == "__main__":
    try:
        integrated_system()
    except KeyboardInterrupt:
        print("[INFO] עצירת המערכת.")
    finally:
        print("[INFO] ניקוי משאבים וסיום.")
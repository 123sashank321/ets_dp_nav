#!/usr/bin/env python3
"""
Analyze navigation bag data
Usage: python3 analyze_bag.py <bag_directory>
"""

import sys
import sqlite3
from pathlib import Path
import json

def analyze_bag(bag_path):
    db_file = list(Path(bag_path).glob("*.db3"))[0]
    conn = sqlite3.connect(str(db_file))
    cursor = conn.cursor()
    
    # Get message counts
    cursor.execute("""
        SELECT topics.name, COUNT(messages.id) 
        FROM messages 
        JOIN topics ON messages.topic_id = topics.id 
        GROUP BY topics.name
    """)
    
    print("Message counts:")
    for topic, count in cursor.fetchall():
        print(f"  {topic}: {count}")
    
    # Get time range
    cursor.execute("SELECT MIN(timestamp), MAX(timestamp) FROM messages")
    start, end = cursor.fetchone()
    duration = (end - start) / 1e9
    print(f"\nDuration: {duration:.2f} seconds")
    
    conn.close()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 analyze_bag.py <bag_directory>")
        sys.exit(1)
    
    analyze_bag(sys.argv[1])

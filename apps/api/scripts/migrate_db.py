"""
Database Migration Script

Run Alembic migrations programmatically.

Usage:
    python apps/api/scripts/migrate_db.py

Environment Variables:
    NEON_CONNECTION_STRING: PostgreSQL connection string for Neon database
"""

import os
import sys
from alembic.config import Config
from alembic import command

def run_migrations():
    """
    Run all pending Alembic migrations.

    Returns:
        bool: True if successful, False otherwise
    """
    # Check for required environment variable
    if not os.getenv("NEON_CONNECTION_STRING"):
        print("ERROR: NEON_CONNECTION_STRING environment variable not set")
        print("See apps/api/.env.example for reference")
        return False

    try:
        # Create Alembic configuration
        alembic_cfg = Config("apps/api/alembic.ini")

        # Run migrations
        print("Running database migrations...")
        command.upgrade(alembic_cfg, "head")

        print("✓ Migrations completed successfully!")
        return True

    except Exception as e:
        print(f"ERROR: Migration failed: {e}")
        return False

if __name__ == "__main__":
    success = run_migrations()
    sys.exit(0 if success else 1)

#!/usr/bin/env python3
"""
Test script to verify Neon Serverless Postgres connection setup.
"""

import os
import sys
from pathlib import Path

# Add the api directory to the path
sys.path.insert(0, str(Path(__file__).parent))

def test_neon_setup():
    """Test the Neon database connection setup."""
    print("Testing Neon Serverless Postgres setup...")

    # Test 1: Check if environment variables are set
    print("\n1. Checking environment variables...")
    neon_connection_string = os.getenv("NEON_CONNECTION_STRING")

    if neon_connection_string:
        print(f"   ✅ NEON_CONNECTION_STRING is set: {neon_connection_string[:50]}...")
    else:
        print("   ❌ NEON_CONNECTION_STRING is not set")
        print("      Please set this in your .env file as:")
        print("      NEON_CONNECTION_STRING=postgresql://username:password@ep-xxx.us-east-1.aws.neon.tech/dbname?sslmode=require")
        return False

    # Test 2: Check if models can be imported without errors
    print("\n2. Testing model imports...")
    try:
        from src.models.user import User
        from src.models.audit_log import AuditLog
        print("   ✅ Models imported successfully")
        print(f"   - User model: {User.__tablename__}")
        print(f"   - AuditLog model: {AuditLog.__tablename__}")
    except ImportError as e:
        print(f"   ❌ Error importing models: {e}")
        return False
    except Exception as e:
        print(f"   ❌ Error with models: {e}")
        return False

    # Test 3: Check if database configuration can be imported
    print("\n3. Testing database configuration...")
    try:
        from src.core.database import engine, Base
        print("   ✅ Database configuration imported successfully")
        print(f"   - Engine type: {type(engine)}")
        print(f"   - Base class: {Base}")
    except ImportError as e:
        print(f"   ❌ Error importing database configuration: {e}")
        return False
    except Exception as e:
        print(f"   ❌ Error with database configuration: {e}")
        return False

    # Test 4: Check if Alembic configuration is correct
    print("\n4. Testing Alembic configuration...")
    try:
        # Check if alembic.ini exists
        alembic_ini_path = Path(__file__).parent / "alembic.ini"
        if alembic_ini_path.exists():
            print("   ✅ alembic.ini file exists")
        else:
            print("   ❌ alembic.ini file not found")
            return False

        # Check if migrations directory exists
        migrations_path = Path(__file__).parent / "migrations"
        if migrations_path.exists():
            print("   ✅ migrations directory exists")
        else:
            print("   ❌ migrations directory not found")
            return False

    except Exception as e:
        print(f"   ❌ Error with Alembic configuration: {e}")
        return False

    print("\n✅ All Neon Serverless Postgres setup tests passed!")
    print("\nTo use Neon:")
    print("1. Install dependencies: pip install -r requirements.txt")
    print("2. Run migrations: alembic upgrade head")
    print("3. The database is now ready for use with Neon Serverless Postgres!")

    return True

if __name__ == "__main__":
    success = test_neon_setup()
    if not success:
        print("\n❌ Neon setup failed. Please fix the issues above.")
        sys.exit(1)
    else:
        print("\n✅ Neon Serverless Postgres is properly configured!")
        sys.exit(0)
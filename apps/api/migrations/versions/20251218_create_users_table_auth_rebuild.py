"""create_users_table_auth_rebuild

Revision ID: 20251218_auth_rebuild
Revises: 20251206_4beab1917973
Create Date: 2025-12-18

Authentication system rebuild - Create users table with experience levels
"""
from alembic import op
import sqlalchemy as sa


# revision identifiers, used by Alembic.
revision = '20251218_auth_rebuild'
down_revision = '20251206_4beab1917973'
branch_labels = None
depends_on = None


def upgrade() -> None:
    """Create users table"""
    # Drop old users table if exists (from previous auth implementation)
    op.execute("DROP TABLE IF EXISTS users CASCADE")

    # Create users table with new schema
    op.create_table(
        'users',
        sa.Column('id', sa.Integer(), nullable=False),
        sa.Column('email', sa.String(), nullable=False),
        sa.Column('password_hash', sa.String(), nullable=False),
        sa.Column('software_level', sa.Enum('beginner', 'intermediate', 'advanced', name='experiencelevel'), nullable=False),
        sa.Column('hardware_level', sa.Enum('beginner', 'intermediate', 'advanced', name='experiencelevel'), nullable=False),
        sa.Column('created_at', sa.DateTime(), nullable=False),
        sa.Column('updated_at', sa.DateTime(), nullable=False),
        sa.PrimaryKeyConstraint('id')
    )

    # Create indexes
    op.create_index(op.f('ix_users_id'), 'users', ['id'], unique=False)
    op.create_index(op.f('ix_users_email'), 'users', ['email'], unique=True)


def downgrade() -> None:
    """Drop users table"""
    op.drop_index(op.f('ix_users_email'), table_name='users')
    op.drop_index(op.f('ix_users_id'), table_name='users')
    op.drop_table('users')
    op.execute("DROP TYPE IF EXISTS experiencelevel")
